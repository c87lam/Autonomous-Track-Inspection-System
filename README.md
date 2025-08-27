# Multi-Sensor Data Acquisition and Processing System
A modular framework for real-time data collection and processing in embedded systems and rail infrastructure, designed to synchronize multi-sensor inputs for defect detection and geometry monitoring.

### 🚀 Core Objectives

- **Real-time data acquisition** from the following sensors:
  - 🎥 **Cameras**: Capture high-resolution, distinct images for training machine learning models targeting **surface defect detection**.
  - 🌐 **LiDAR**: Perform high-frequency scans to compute **track geometry**, including **gauge width** and **height deviations**.
  - 🧭 **INS (Inertial Navigation System)**: Derive **trajectory and versine** per segment for curvature tracking.
  - 🌡️ **Thermometers**: Monitor ambient temperature conditions.

### 🧠 System Structure

```text
├── include/       # Header files
├── src/           # Implementation
├── third_party    # Placeholder for proprietary sensor SDKs
├── install.txt    # Installation/setup instructions
├── run.sh         # Script to build & run
└── README.md      # Project overview, usage guide, and documentation
```

## 🛠️ Dependencies

- Qt 5 – GUI framework
- TBB – Parallel execution
- OpenCV – Image processing
- yaml-cpp – YAML configuration
- Vendor SDKs (not included):
  - Camera SDK
  - INS SDK
  - LiDAR SDK

## ⚙️ Build & Execution

All operations are managed via the `run.sh` script.

### 📦 Script Usage

```bash
./run.sh [OPTIONS]

-n,   --newbuild        Clean build (removes generated folders and files)
-b,   --buildtype       Specify build type 'debug' or 'release' (default: release)
-cam, --camera_dir      Path to Camera SDK (must include 'include' and 'lib')
-imu, --imu_dir         Path to IMU SDK
-lidar, --lidar_dir     Path to LiDAR SDK
-h,   --help            Show usage instructions
```

### 🧪 Example Usage

```bash
# Clean build and run in release mode
./run.sh -n -cam /path/to/camera_sdk -imu /path/to/imu_sdk -lidar /path/to/lidar_sdk

# Debug build
./run.sh -b debug -cam /path/to/camera_sdk -imu /path/to/imu_sdk -lidar /path/to/lidar_sdk

# Clean build in debug mode
./run.sh -n -b debug -cam /path/to/camera_sdk -imu /path/to/imu_sdk -lidar /path/to/lidar_sdk
```

## 📁 Configuration

System behavior is controlled via YAML configuration files which define:

- Sampling rates and delays
- IP addresses, ports, and serial communication parameters
- Output folders and logging preferences

YAML files are validated at runtime. Missing or malformed values will trigger fallback defaults or runtime errors.

## 🔐 Disclaimer
```text
This project is provided for personal and educational purposes only.  
Any unauthorized use, reproduction, or distribution of the code in this repository is strictly prohibited.  

All proprietary sensor SDKs, device paths, IP addresses, and credentials have been removed.  
The project will not run as-is without the original hardware and SDKs.  
This repository is intended for demonstration and portfolio purposes only and does not expose any confidential company data.
```
