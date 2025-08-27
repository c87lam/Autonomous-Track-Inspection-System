#ifndef SYNC_HPP
#define SYNC_HPP

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <utility>
#include <fstream>
#include <memory>
#include <string>
#include <filesystem>
#include <variant>
#include <unordered_set>
#include <mutex>

#include "camera/func.hpp"
#include "ins/func.hpp"
#include "lidar/func.hpp"
#include "thermometer/func.hpp"

#include "pools/unified_pool.hpp"
#include "../param.hpp"

struct SyncData {
    DataType::INS::Ptr ins;

    // Location to ptr pair
    std::map<Props::Location, DataType::Lidar::Ptr> lidar;

    // Sensor num to ptr pair
    std::map<std::string, DataType::Thermometer::Ptr> thermometer;
    std::map<std::string, DataType::Camera::Ptr> camera;
};

struct CameraGroup {
    double avg_timestamp;
    std::vector<DataType::Camera::Ptr> cameras;

    CameraGroup(double ts, std::vector<DataType::Camera::Ptr> cams)
      : avg_timestamp(ts), cameras(std::move(cams)) {}
};

class SyncManager {
    private:
        using Thermometer = std::vector<ThermometerData::Ptr>;
        using Lidar = std::vector<LidarData::Ptr>;
        using Camera = std::vector<CameraData::Ptr>;
        using INS = std::vector<INSData::Ptr>;

        using ThermometerVec = std::vector<Thermometer>;
        using LidarVec = std::vector<Lidar>;
        using CameraVec = std::vector<Camera>;
        using INSVec = std::vector<INS>;

        using SensorType = std::variant<Thermometer, Camera, Lidar, INS>;

        mutable std::mutex sync_mutex;
        const int NUM_SENSORS = 4;
        std::unordered_set<Props::Sensors> seen;
        int csv_count;

    public:
        LidarVec lidar;
        ThermometerVec thermometer;
        CameraVec camera;
        std::map<int, CameraVec> camera_map;
        INSVec ins;

        std::filesystem::path base_path;

        SyncManager() {
            std::filesystem::path file_path = params.data_storage;
            file_path /= "Merged Data";
            base_path = file_path;
            csv_count = 0;

            Props::ensure_dir(base_path.string());
        }

        ~SyncManager() {
            clear_data();
        }

        std::string create_header(const SyncData &data) {
            std::ostringstream out;
            out << "INS IP: " << data.ins->name << "\n";

            for (const auto &[name, _] : data.thermometer) {
                out << "Thermometer IP: " << name << " (°C),\n";
            }

            for (const auto &[name, _] : data.camera) {
                out << "Camera IP: " << name << "\n";
            }

            std::string &name_left = data.lidar.at(Props::Location::LEFT)->name;
            std::string &name_right = data.lidar.at(Props::Location::RIGHT)->name;
            out << "Lidar IP: " << name_left << " Located on LEFT \n";
            out << "Lidar IP: " << name_right << " Located on RIGHT \n";

            out << "Timestamp (" << Props::TIME_FMT << "),"
                << "Thermometer 1 (°C),"
                << "Thermometer 2 (°C),"
                << "X_left [m],"
                << "Z_left [m],"
                << "Intensity_left,"
                << "Peakwidth_left [m],"
                << "X_right [m],"
                << "Z_right [m],"
                << "Intensity_right,"
                << "Peakwidth_right [m],"
                << "Horizontal Speed (m/s),"
                << "Vertical Speed (m/s),"
                << "Roll (deg),"
                << "Pitch (deg),"
                << "Yaw (deg),"
                << "Latitude,"
                << "Longitude,"
                << "Altitude (m),"
                << "Gyroscope X (rad/s),"
                << "Gyroscope Y (rad/s),"
                << "Gyroscope Z (rad/s),"
                << "Accelerometer X (m/s²),"
                << "Accelerometer Y (m/s²),"
                << "Accelerometer Z (m/s²),"
                << "Magnetometer X (µT),"
                << "Magnetometer Y (µT),"
                << "Magnetometer Z (µT),"
                << "Image Left 1,"
                << "Image Left 2,"
                << "Image Right 1,"
                << "Image Right 2"
                << "\n";
            return out.str();
        }

        SensorType what_sensor(Props::Sensors sensor){
            switch(sensor){
                case Props::Sensors::LIDAR:
                    return Lidar{};
                case Props::Sensors::CAM:
                    return Camera{};
                case Props::Sensors::TEMP:
                    return Thermometer{};
                case Props::Sensors::INS:
                    return INS{};
                default:
                    throw std::runtime_error("Unknown sensor type");
            }
        }

        void clear_data(){
            thermometer.clear();
            lidar.clear();
            camera.clear();
            ins.clear();
            camera_map.clear();
            seen.clear();
        }

        // Function to skip the first comma text (which is the timestamp)
        std::string truncate_timestamp(std::string &text) {
            size_t pos = text.find(',');
            std::string data = (pos != std::string::npos) ? text.substr(pos + 1) : text;

            // Remove trailing whitespace (spaces, tabs, newlines, carriage returns)
            data.erase(std::remove_if(data.begin(), data.end(), ::isspace), data.end());

            return data;
        }

        template <typename Manager>
        void process_merge_data(Props::Sensors sensor, const TaskFuncs::Set &synced, const std::shared_ptr<Manager> &mgr){
            std::lock_guard<std::mutex> lock(sync_mutex);
            for (const auto &sensor_info : synced){
                const auto& sensorNum = mgr->get_sensor();
                auto data_opt = sensorNum[sensor_info.id]->get_data(sensor_info.timestamp, true);
                if (!data_opt || data_opt->empty()){
                    continue;
                }

                auto result = what_sensor(sensor);
                result = *data_opt;

                if (sensor == Props::Sensors::CAM){
                    camera_map[sensor_info.id].push_back(std::get<Camera>(result));
                } else if (sensor == Props::Sensors::LIDAR) {
                    lidar.push_back(std::get<Lidar>(result));
                } else if (sensor == Props::Sensors::TEMP){
                    thermometer.push_back(std::get<Thermometer>(result));
                } else if (sensor == Props::Sensors::INS){
                    ins.push_back(std::get<INS>(result));
                }
            }

            seen.insert(sensor);
            if (seen.size() == NUM_SENSORS){
                std::cout << "all sensors have been seen" << std::endl;
                camera.resize(camera_map.size());

                for (const auto &[id, vec] : camera_map) { // Flattening camera
                    size_t total_size = 0;
                    for (const auto &group : vec){
                        total_size += group.size();
                    }

                    Camera &flat_cam = camera[id];
                    flat_cam.reserve(total_size);

                    for (auto group : vec){
                        flat_cam.insert(flat_cam.end(), group.begin(), group.end());
                    }
                }

                auto camera_pool = camera_median(camera);

                DataAdhoc::Pool<DataType::INS> data_ins;
                std::vector<DataAdhoc::Pool<DataType::Lidar>> lidar_data(lidar.size());
                std::vector<DataAdhoc::Pool<DataType::Thermometer>> thermometer_data(thermometer.size());
                std::vector<DataAdhoc::Pool<CameraGroup>> camera_data = {camera_pool};

                // Create a map to hold the dataset
                std::map<double, SyncData> dataset;

                for (size_t i=0; i<ins.size(); i++){
                    for (const auto &data: ins[i]){
                        DataType::INS::Ptr ptr = std::make_shared<DataType::INS>(data->time, data->name, data->id, std::move(data->ins_data));
                        data_ins.emplace(data->time, ptr);
                        dataset[data->time].ins = ptr;
                    }
                }

                for (size_t i=0; i<thermometer.size(); i++){
                    std::string name = "Thermometer " + std::to_string(i);
                    for (const auto &data: thermometer[i]){
                        thermometer_data[i].emplace(data->time_data, data->time_data, data->temperature, name);
                    }
                }

                for (size_t i=0; i<lidar.size(); i++){
                    for (const auto &data: lidar[i]){
                        lidar_data[i].emplace(data->time_data, data->time_data, data->name,
                                              data->orig_rows, data->transformed_rows, data->cnt_dir);
                    }
                }

                // Data type is std::map<double, std::map<std::string, Ptr>>
                auto ins_thermometer = Matcher::match<DataType::Thermometer, DataType::Thermometer::Ptr, DataType::INS>(data_ins, thermometer_data);
                auto ins_lidar = Matcher::match<DataType::Lidar, DataType::Lidar::Ptr, DataType::INS>(data_ins, lidar_data);
                auto ins_camera = Matcher::match_cam<CameraGroup, std::shared_ptr<CameraGroup>, DataType::INS>(data_ins, camera_data);

                for (const auto &[ins_ts, mp]: ins_thermometer){
                    auto &entry = dataset[ins_ts];
                    for (const auto&[sensor_name, ptr]: mp){
                        entry.thermometer[sensor_name] = ptr;
                    }
                }

                for (const auto &[ins_ts, mp]: ins_lidar){
                    auto &entry = dataset[ins_ts];
                    for (const auto&[sensor_name, ptr]: mp){
                        entry.lidar[ptr->cnt_dir] = ptr;
                    }
                }

                // std::unordered_map<double, CameraGroup>
                for (const auto &[ins_ts, cam_struct]: ins_camera){
                    auto &entry = dataset[ins_ts];
                    for (const auto& ptr: cam_struct->cameras){
                        entry.camera[ptr->name] = ptr;
                    }
                }

                std::string file = "merge " + std::to_string(csv_count) + ".csv";
                csv_count++;
                print_data(file, dataset);

                std::cout << "Done, program finished" << std::endl;
                clear_data();
            }
        }

        double get_median(std::vector<DataType::Camera::Ptr> &num){
            size_t n = num.size();
            if (n%2 != 0)
                return num[n/2]->time_sec;
            
            return (num[n/2 - 1]->time_sec + num[n/2]->time_sec) / 2.0;
        }

        DataAdhoc::Pool<CameraGroup> camera_median(CameraVec& camera){
            size_t min_size = std::numeric_limits<size_t>::max();
            const double TOLERANCE = 0.3;
            for (const auto& cam : camera)
                min_size = std::min(min_size, cam.size());

            DataAdhoc::Pool<CameraGroup> camera_pool;

            for (size_t i=0; i<min_size; i++){
                std::vector<DataType::Camera::Ptr> row;
                for (int id = 0; id <camera_map.size(); ++id) {
                    auto ptr = std::make_shared<DataType::Camera>(camera[id][i]->time_data, 
                        camera[id][i]->name, camera[id][i]->dir, camera[id][i]->image.clone());
                    row.push_back(std::move(ptr));
                }
    
                std::sort(row.begin(), row.end(), [](const auto &a, const auto &b){
                    return a->time_sec < b->time_sec;
                });

                double skew = row[camera_map.size()-1]->time_sec - row[0]->time_sec;
                if (skew >= TOLERANCE)
                    continue;

                double median = get_median(row);
                camera_pool.emplace(median, median, std::move(row));
            }
            return camera_pool;
        }

        void print_data(std::string &filename, std::map<double, SyncData> &dataset){
            std::filesystem::path store_folder = base_path / filename;
            bool needs_header = !std::filesystem::exists(store_folder.string());

            std::filesystem::path image_path = params.data_storage + "/Images";

            std::ofstream file(store_folder.string(), std::ios::app);
            if (!file.is_open()){
                std::cout << "Unable to open file" << std::endl;
                return;
            }

            if (needs_header){
                file << create_header(dataset.begin()->second); 
            }

            for (const auto &[timestamp, mp]: dataset){
                std::stringstream oss;
                oss << std::to_string(timestamp) << ", ";

                /* Thermometer Data */
                for (const auto& [sensor_name, ptr]: mp.thermometer){
                    std::ostringstream temp_stream;
                    temp_stream << std::fixed << std::setprecision(1) << ptr->temperature;
                    oss << temp_stream.str() << ", ";
                }

                /* INS data */
                std::string ins_info = mp.ins->ins_data;
                oss << truncate_timestamp(ins_info) << ", ";

                /* Lidar data (printing original rows) */
                if (mp.lidar.count(Props::Location::LEFT) == 0 || mp.lidar.count(Props::Location::RIGHT) == 0){ 
                    continue;
                }

                auto left_ptr = mp.lidar.at(Props::Location::LEFT);
                auto right_ptr = mp.lidar.at(Props::Location::RIGHT);

                if (!left_ptr || !right_ptr){
                    continue;
                }

                auto &left = left_ptr->transformed_rows;
                auto &right = right_ptr->transformed_rows;

                size_t min_rows = std::min(left.size(), right.size());
                for (size_t i=0; i<min_rows; i++){
                    std::ostringstream row;
                    row << oss.str();
                    row << truncate_timestamp(left[i]) << truncate_timestamp(right[i]) << "\n";

                    for (const auto &[sensor_name, ptr]: mp.camera){
                        double timestamp_folder = std::floor(ptr->time_sec);
                        std::filesystem::path image_folder = image_path / Props::format_timestamp(timestamp_folder)
                                            / Props::loc2str(ptr->dir) / ptr->name / ptr->time_str;
                        row << image_folder << ", ";
                    }
                    row << "\n";
                    file << row.str();
                }
            }
        }
};

#endif