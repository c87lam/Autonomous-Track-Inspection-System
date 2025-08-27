#include "utils.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

// -------- For handling information on sensor location e.t.c -------- //
SensorTransform::SensorTransform() {
  translation.setZero();
  rotation_rpy.setZero();
  axis_remap.setIdentity();
  transform.setIdentity();
}

Eigen::Matrix3d SensorTransform::axis_remap_func(const YAML::Node &remap_node) {
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();

  std::vector<std::string> sensor_axes = {"x_sensor", "y_sensor", "z_sensor"};
  for (int idx = 0; idx < 3; ++idx) {
    const auto &node = remap_node[sensor_axes[idx]];
    int axis = node["axis"].as<int>(); // 0 = x, 1 = y, 2 = z
    int sign = node["sign"].as<int>(); // +1 or -1
    R(axis, idx) = sign;
  }

  return R;
}

Eigen::Matrix4d SensorTransform::invert_transform(const Eigen::Matrix4d &T) {
  Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();

  // Extract rotation (top-left 3×3 part)
  Eigen::Matrix3d R = T.topLeftCorner<3, 3>();

  // Extract translation (top-right 3×1 part)
  Eigen::Vector3d t = T.topRightCorner<3, 1>();

  // Compute inverse transformation
  T_inv.topLeftCorner<3, 3>() = R.transpose();       // R^T
  T_inv.topRightCorner<3, 1>() = -R.transpose() * t; // -R^T * t

  return T_inv;
}

SensorTransform SensorTransform::load_frm_yaml(const YAML::Node &sensor_node) {
  SensorTransform st;

  st.name = sensor_node["name"].as<std::string>();
  st.parent_frame = sensor_node["parent_frame"].as<std::string>();

  if (sensor_node["sensor_location"]) {
    int code = sensor_node["sensor_location"].as<int>();
    st.sensor_location = Props::parse_location(code);
  }

  if (sensor_node["engineer_notes"]) {
    st.engineer_notes = sensor_node["engineer_notes"].as<std::string>();
  }

  st.translation.x() = sensor_node["translation"]["x"].as<double>();
  st.translation.y() = sensor_node["translation"]["y"].as<double>();
  st.translation.z() = sensor_node["translation"]["z"].as<double>();

  st.rotation_rpy.x() = sensor_node["angular"]["roll"].as<double>();
  st.rotation_rpy.y() = sensor_node["angular"]["pitch"].as<double>();
  st.rotation_rpy.z() = sensor_node["angular"]["yaw"].as<double>();

  st.axis_remap = SensorTransform::axis_remap_func(sensor_node["axis_remap"]);

  // Parent to sensor
  Eigen::Matrix3d R_rpy = Props::deg_rpy2rot(
      st.rotation_rpy.x(), st.rotation_rpy.y(), st.rotation_rpy.z());

  // Transformation from parent to sensor_base
  Eigen::Matrix4d T_psb;
  T_psb.setIdentity();
  T_psb.topLeftCorner<3, 3>() = R_rpy;
  T_psb.topRightCorner<3, 1>() = st.translation;

  // Transformation from world to sensor_base
  Eigen::Matrix4d T_wsb;
  T_wsb.setIdentity();
  T_wsb.topLeftCorner<3, 3>() = st.axis_remap;

  // Transformation from world to parent
  st.transform = T_wsb * SensorTransform::invert_transform(T_psb);

  st.valid = true;
  return st;
}

void SensorTransform::write_info(const SensorTransform::Map &sensors,
                                 const std::string &filename) {
  if (sensors.empty()) {
    std::cerr << "[SensorTransform] No sensor data available to write.\n";
    return;
  }

  std::ofstream out(filename);
  if (!out) {
    throw std::runtime_error("Failed to open output file: " + filename);
  }

  for (const auto &[name, st] : sensors) {
    if (!st.valid)
      continue;

    out << "==================================================\n";
    out << "Measurements are in meters [m]. Angles in Degrees\n";
    out << "Sensor Name      : " << name << "\n";
    out << "Parent Frame     : " << st.parent_frame << "\n";
    out << "Sensor Location  : " << Props::loc2str(st.sensor_location) << "\n";
    if (!st.engineer_notes.empty())
      out << "Engineer Notes   : " << st.engineer_notes << "\n";

    out << std::fixed << std::setprecision(4);

    out << "\n-- Translation (m) --\n";
    out << "  x: " << st.translation.x() << "\n";
    out << "  y: " << st.translation.y() << "\n";
    out << "  z: " << st.translation.z() << "\n";

    out << "\n-- Rotation RPY (deg) --\n";
    out << "  roll : " << st.rotation_rpy.x() << "\n";
    out << "  pitch: " << st.rotation_rpy.y() << "\n";
    out << "  yaw  : " << st.rotation_rpy.z() << "\n";

    out << "\n-- Axis Remap Matrix --\n";
    out << st.axis_remap << "\n";

    out << "\n-- Transformation Matrix (4x4) Parent Frame-to-sensor--\n";
    out << st.transform << "\n\n";
  }

  out.close();
}
