#include "sensors/camera/func.hpp"
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <optional>
#include <shared_mutex>
#include <sstream>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <unordered_map>

namespace {
static std::pair<int, double>
chan_format_and_scale(const std::string &px_format) {
  // Define the mapping of pixel formats to OpenCV types and scaling factors
  static const std::unordered_map<std::string, std::pair<int, double>>
      img_format = {
          {"Mono8", {CV_8UC1, 1.0}},       {"BayerRG8", {CV_8UC1, 1.0}},
          {"BayerGB8", {CV_8UC1, 1.0}},    {"BGR8", {CV_8UC3, 1.0}},
          {"BGR8Packed", {CV_8UC3, 1.0}},  {"Mono16", {CV_16UC1, 1.0}},
          {"Mono12", {CV_16UC1, 16.0}},    {"BayerRG12", {CV_16UC1, 16.0}},
          {"BayerGB12", {CV_16UC1, 16.0}}, {"Mono10", {CV_16UC1, 64.0}},
          {"BayerRG10", {CV_16UC1, 64.0}}, {"BayerGB10", {CV_16UC1, 64.0}},
          {"BGR12", {CV_16UC3, 1.0}}};

  // Attempt to find the format in the map
  auto it = img_format.find(px_format);
  if (it != img_format.end()) {
    return it->second;
  } else {
    // Throw an error if the pixel format is unsupported
    throw std::runtime_error("Unsupported pixel format: " + px_format);
  }
}

static std::pair<int, int> scale_maintainer(int base_width, int base_height,
                                            int target_width,
                                            int target_height) {
  // Calculate scaling factors
  double width_scale = static_cast<double>(target_width) / base_width;
  double height_scale = static_cast<double>(target_height) / base_height;

  // Use the smaller scale to maintain the aspect ratio
  double scale = std::min(width_scale, height_scale);

  // Calculate new dimensions
  int new_width = static_cast<int>(std::round(base_width * scale));
  int new_height = static_cast<int>(std::round(base_height * scale));

  return {new_width, new_height};
}
} // namespace

namespace DataDevice {
Camera::Camera(FlowControl::Ptr ctrl, int id, CameraDevice *device,
               TaskPool::Ptr tp)
    : BaseClass::Sensor(Props::Sensors::CAM, create_name(device), ctrl, id),
      device(device), oc(std::make_shared<OverlapCheck>()) {
  initialize();
  task_pool = tp;

  dir = transform_info["camera"][get_ip(device)].sensor_location;
}

Camera::~Camera() {
  stop();
  flush_all(storage);
  terminate_thread();
}

std::string Camera::get_ip(CameraDevice *_device) {
  if (_device->GetTLType() == "GEV") {
    // Retrieve the IP address safely
    auto ip_addy = _device->GetRemoteNode("GevCurrentIPAddress")->GetInt();

    // Format the IP address into a string
    std::ostringstream ip_stream;
    ip_stream << ((ip_addy & 0xff000000) >> 24) << "."
              << ((ip_addy & 0x00ff0000) >> 16) << "."
              << ((ip_addy & 0x0000ff00) >> 8) << "." << (ip_addy & 0x000000ff);

    return ip_stream.str();
  }

  return "";
}

std::string Camera::create_name(CameraDevice *_device) {
  try {
    std::string name_base = std::string(_device->GetModel());
    name_base += "-" + get_ip(_device);

    return name_base;
  } catch (CameraException &ex) {
    std::string error_msg =
        "Error creating camera name: " + std::string(ex.GetErrorDescription());
    error_log(error_msg);
    return "Unknown-Camera"; // Fallback default name
  }
}

// ------------------ Initializers ------------------ //

void Camera::get_default_size() {
  bounds.max_width = device->GetRemoteNode("Width")->GetIntMax();
  bounds.min_width = device->GetRemoteNode("Width")->GetIntMin();

  bounds.max_height = device->GetRemoteNode("Height")->GetIntMax();
  bounds.min_height = device->GetRemoteNode("Height")->GetIntMin();

  device->GetRemoteNode("Width")->SetInt(bounds.max_width);
  device->GetRemoteNode("Height")->SetInt(bounds.max_height);

  update_size();
}

void Camera::update_size() {

  if (bounds.max_width == params.cam.image_width &&
      bounds.max_height == params.cam.image_height) {
    success_log("Default Image size Maintainted!");

    return;
  }

  auto new_dims =
      scale_maintainer(bounds.max_width, bounds.max_height,
                       params.cam.image_width, params.cam.image_height);

  std::ostringstream out;
  out << "\nResizing Image Dimensions:\n";
  out << "Target image height: " << new_dims.second << "\n";
  out << "Target image width: " << new_dims.first;

  success_log(out.str());
  device->GetRemoteNode("Width")->SetInt(new_dims.first);
  device->GetRemoteNode("Height")->SetInt(new_dims.second);
}

void Camera::set_px_format() {
  auto available_px_format = [&](std::string &px_format) {
    auto node_list = device->GetRemoteNode("PixelFormat")->GetNodeList();
    return node_list->GetNodePresent(px_format) &&
           node_list->GetNode(px_format)->IsReadable();
  };

  // List of supported pixel formats in priority order
  const std::vector<std::string> formats = {
      "Mono8",     "BayerRG8",  "BayerGB8",  "BGR8",      "BGR8Packed",
      "Mono16",    "Mono12",    "BayerRG12", "BayerGB12", "Mono10",
      "BayerRG10", "BayerGB10", "BGR12"};

  // Iterate through the pixel formats and set the first available one
  for (const auto &format : formats) {
    if (available_px_format(format)) {
      device->GetRemoteNode("PixelFormat")->SetString(format);
      std::string px_form = std::string(format);
      std::tie(cv_type, scaling) = chan_format_and_scale(px_form);
      return;
    }
  }

  throw std::runtime_error(
      "None of the supported pixel formats are available.");
}

void Camera::set_gain_exposure() {
  if (params.cam.exposure > 0 &&
      device->GetRemoteNode("ExposureAuto")->IsWriteable()) {
    device->GetRemoteNode("ExposureAuto")->SetString("Off");
    device->GetRemoteNode("ExposureTime")->SetDouble(params.cam.exposure);

  } else if (device->GetRemoteNode("ExposureAuto")->IsWriteable())
    device->GetRemoteNode("ExposureAuto")->SetString("Continuous");

  if (params.cam.gain > 0 && device->GetRemoteNode("GainAuto")->IsWriteable()) {
    device->GetRemoteNode("GainAuto")->SetString("Off");
    device->GetRemoteNode("Gain")->SetDouble(params.cam.gain);

  } else if (device->GetRemoteNode("GainAuto")->IsWriteable())
    device->GetRemoteNode("GainAuto")->SetString("Continuous");
}

void Camera::configure() {
  try {
    device->GetRemoteNode("AcquisitionStop")->Execute();

    get_default_size(); // default sizing

    set_px_format(); // setting the px format type. Should handle color/BW

    device->GetRemoteNode("TriggerMode")->SetValue("Off");

    set_gain_exposure();

  } catch (CameraException &ex) {
    error_log("Error configuring:" +
              std::string(ex.GetErrorDescription()));
    std::exit(0);

  } catch (std::exception &e) {
    error_log("Error configuring (General error):" + std::string(e.what()));
    std::exit(0);
  }
}

void Camera::data_stream_setup() {
  CameraDataStreamList *dslist = device->GetDataStreams();
  dslist->Refresh();

  if (dslist->size() == 0)
    throw std::runtime_error("No datastreams found for device.");

  data_stream = dslist->begin()->second;
  data_stream->Open();

  if (params.cam.num_buffer <= 0)
    return;

  // setup buffer list
  CameraBufferList *buff_list = data_stream->GetBufferList();

  // using smart pointer to create buffer
  for (int idx = 0; idx < params.cam.num_buffer; idx++) {
    auto buffer = std::make_unique<CameraBuffer>();
    buff_list->Add(buffer.get());
    buffer->QueueBuffer();
    buffers.push_back(std::move(buffer));
  }
}

void Camera::initialize() {
  configure();

  data_stream_setup();

  success_log("Initialization Complete.");

  work_thread = std::thread(&Camera::run, this);
}

// ------------------ Trigger controls ------------------ //
void Camera::start() {
  if (acquire_started)
    return;

  // Start camera acquisition
  data_stream->StartAcquisitionContinuous();
  device->GetRemoteNode("AcquisitionStart")->Execute();
  acquire_started = true;

  success_log("Data Acquisition started");
}

void Camera::pause() {
  if (!acquire_started)
    return;

  device->GetRemoteNode("AcquisitionStop")->Execute();
  data_stream->StopAcquisition();
  acquire_started = false;

  success_log("Data Acquisition Paused.");
}

void Camera::stop() {
  if (is_stopped)
    return;

  is_stopped = true;

  if (data_stream) {
    try {
      data_stream->StopAcquisition();
      success_log("Stopped data stream.");
    } catch (CameraException &ex) {
      std::string error_msg = "Error stopping data stream: " +
                              std::string(ex.GetErrorDescription());
      error_log(error_msg);
    }
  }

  if (device) {
    try {
      device->Close();
      success_log("Camera device closed.");
    } catch (CameraException &ex) {
      std::string error_msg = "Error closing camera device: " +
                              std::string(ex.GetErrorDescription());
      error_log(error_msg);
    }
  }

  success_log("Data Acquisition Stopped.");
}

// ------------------ Data Processing and Alert ------------------ //
CameraData::Ptr Camera::process_data(CameraBuffer *buffer) {
  try {
    void *img_ptr = buffer->GetMemPtr();

    if (!img_ptr)
      throw std::runtime_error("Invalid buffer memory or size.");

    // storing image information
    auto height = static_cast<int>(buffer->GetHeight());
    auto width = static_cast<int>(buffer->GetWidth());

    if (!storage_val_updated) {
      storage_val_updated = true;
      storage_height = static_cast<int>(height * params.cam.size_compression);
      storage_width = static_cast<int>(width * params.cam.size_compression);
    }

    cv::Mat image;
    cv::Mat raw_image(
        height, width, cv_type,
        (char *)((bo_uint64)(buffer->GetMemPtr()) + buffer->GetImageOffset()));

    if (scaling != 1.0) {
      raw_image.convertTo(image, cv_type, scaling);
    } else {
      raw_image.copyTo(image);
    }
    // resize image to save size
    cv::Mat resized_img;
    try {
      cv::resize(image, resized_img, cv::Size(storage_width, storage_height), 0,
                 0, cv::INTER_AREA);
    } catch (const cv::Exception &e) {
      throw std::runtime_error("OpenCV resize error: " + std::string(e.what()));
    }

    image = std::move(resized_img);
    if ((params.cam.overlap_ratio >= 0.0 && params.cam.overlap_ratio <= 1.0)) {
      if (!oc->should_save(image))
        return nullptr;
    }

    CameraData::Ptr data = std::make_shared<CameraData>();
    data->image = std::move(image);
    data->dir = dir;
    return data;

  } catch (const std::exception &e) {
    error_log("Error processing image data: " + std::string(e.what()));
    return nullptr;
  }
}

void Camera::get_packet() {
  try {
    CameraBuffer *buffer = nullptr;
    auto [time_double, string_stamp] = Props::create_timestamp();
    buffer = data_stream->GetFilledBuffer(params.cam.buffer_delay);

    if (buffer == nullptr) {
      std::string msg =
          "Buffer Timeout after " + std::to_string(params.cam.buffer_delay);
      error_log(msg);
      return;
    }

    if (buffer->GetIsIncomplete()) {
      error_log("Image is incomplete");
      buffer->QueueBuffer();

      return;
    }

    CameraData::Ptr new_data = process_data(buffer);
    buffer->QueueBuffer();

    if (!new_data)
      return;

    // Update the total data information
    new_data->time_data = time_double;
    new_data->timestamp = string_stamp;
    new_data->name = get_sensor_name();

    // add to task queue
    add_to_task(storage, new_data, time_double, params.cam.batch_rate);

  } catch (const std::exception &e) {
    error_log("Critical error in get_packet(): " + std::string(e.what()));
    return;
  }
}

std::optional<std::vector<CameraData::Ptr>> Camera::get_data(double timestamp) {
  return extract_data_at(storage, timestamp);
}

void Camera::run() {
  success_log("Start of Dedicated Thread.");

  while (!ctrl->get_state(FlowControl::Type::STOP)) {
    {
      std::unique_lock<std::mutex> lock(ctrl->mtx);
      ctrl->cv.wait(lock, [&] {
        return ctrl->get_state(FlowControl::Type::RUN) || break_condition();
      });
    }

    if (ctrl->get_state(FlowControl::Type::STOP)) {
      pause();
      break;
    }

    if (ctrl->get_state(FlowControl::Type::RUN)) {
      start();
      get_packet();
    }

    if (ctrl->get_state(FlowControl::Type::PAUSE))
      pause();
  }

  stop(); // stopping system
}
} // namespace DataDevice

// ----------------------------------- //
// -------------- Manager ------------ //
// ----------------------------------- //
int CamManager::camera_counter = 0;

CamManager::CamManager(FlowControl::Ptr ctrl, TaskPool::Ptr tp)
    : BaseClass::Manager(Props::Sensors::CAM) {
  initialize(ctrl, tp);
}

int CamManager::compression_scale(int scale, bool is_png) {
  scale = std::clamp(scale, 1,
                     10); // Ensure scale is within the valid range [1, 10]

  if (is_png) {
    return scale - 1; // Direct mapping (0-9)
  } else {
    return 100 - (scale - 1) * 10; // Maps [1,10] to [100, 10]
  }
}

void CamManager::initialize_img_params() {
  if (params.cam.img_typ == "png") {
    img_params = {cv::IMWRITE_PNG_COMPRESSION,
                  compression_scale(params.cam.quality_compression, true)};
    success_log("PNG Image Format used.");

  } else if (params.cam.img_typ == "jpg" || params.cam.img_typ == "jpeg") {
    img_params = {cv::IMWRITE_JPEG_QUALITY,
                  compression_scale(params.cam.quality_compression, false)};

    success_log("JPEG Image Format used.");
  } else {
    std::ostringstream out;
    out << "Error: Unsupported format '" << params.cam.img_typ
        << "'. Use 'png' or 'jpg'." << std::endl;

    error_log(out.str());
  }
}

void CamManager::initialize(FlowControl::Ptr ctrl, TaskPool::Ptr tp) {
  if (!cameras.empty()) {
    warn_log("Already discovered");
    return;
  }

  CameraSystemList *syslist = CameraSystemList::GetInstance();
  syslist->Refresh();

  for (auto sys_iter = syslist->begin(); sys_iter != syslist->end();
       sys_iter++) {
    system = sys_iter->second;
    system->Open();

    ilist = system->GetInterfaces();
    ilist->Refresh(params.cam.refresh_delay);

    for (auto ilist_iter = ilist->begin(); ilist_iter != ilist->end();
         ilist_iter++) {
      interface = ilist_iter->second;
      interface->Open();

      device_list = interface->GetDevices();
      device_list->Refresh(params.cam.refresh_delay);

      for (auto dev_iter = device_list->begin(); dev_iter != device_list->end();
           dev_iter++) {

        std::string msg = "Error opening device ";
        if (!dev_iter->second)
          continue;

        try {
          dev_iter->second->Open();
          auto camera = std::make_unique<DataDevice::Camera>(
              ctrl, camera_counter++, dev_iter->second, tp);
          cameras.push_back(std::move(camera));
        } catch (CameraException &ex) {
          msg += std::string(ex.GetErrorDescription());
          error_log(msg);

          std::exit(0);
        } catch (std::exception &err) {
          msg += std::string(err.what());
          error_log(msg);
          std::exit(0);
        }
      }
    }
  }

  if (cameras.empty()) {
    error_log("Cameras not found!!");
  }

  set_num_count(cameras.size()); // set total number of sensor found
  initialize_img_params();
  success_log("All Cameras Initialized: " + std::to_string(cameras.size()));
}

CamManager::Ptr CamManager::create(FlowControl::Ptr ctrl, TaskPool::Ptr tp) {
  auto new_cm = std::make_shared<CamManager>(ctrl, tp);
  LOG_INFO("Registering task.");
  tp->register_manager(Props::Sensors::CAM, new_cm);
  LOG_INFO("Camera Manger Task Registered.");

  return new_cm;
}

void CamManager::process_data(const TaskFuncs::Set &synced) {
  for (const auto &cam_info : synced) {
    auto data_opt = cameras[cam_info.id]->get_data(cam_info.timestamp);
    if (!data_opt || data_opt->empty())
      continue;

    const auto &data_vec = *data_opt;

    // Build structured folder path: base_dir / timestamp / direction / name
    std::filesystem::path store_folder;
    {
      double folder_ts = std::floor(cam_info.timestamp);
      store_folder = base_dir / Props::format_timestamp(folder_ts) /
                     Props::loc2str(data_vec.front()->dir) /
                     data_vec.front()->name;

      Props::ensure_dir(store_folder.string());
    }

    for (auto &data : data_vec) {
      if (data->time_data < params.init_complete_time)
        continue;

      std::filesystem::path file_path =
          store_folder / (data->timestamp + ".png");
      try {
        cv::imwrite(file_path.string(), data->image, img_params);
      } catch (const cv::Exception &ex) {
        std::ostringstream out;
        out << "Error saving image to '" << file_path.string()
            << "' due to: " << ex.what() << std::endl;
        error_log(out.str());
      }
    }
  }
}
