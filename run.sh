#!/bin/bash

# usage guide
usage() {
    cat <<EOF
Usage: $0 [OPTIONS]

Description:
    Script is used to build, clean, and run processes for the Autonomous Track Inspection System project.
    Supports building in different configurations (debug/release) and running/generating the executables.

Options:
    -n,     --newbuild       Clean build (removes generated folders and files)
    -b,     --buildtype      Specify build type 'debug' or 'release' (default: release)
    -cam,   --camera_dir     Main directory location for camera SDK folders (include and lib)
    -imu,   --imu_dir        Main directory location for IMU SDK folders
    -lidar, --lidar_dir      Main directory location for LiDAR SDK folders
    -h,     --help           Display this usage guide

Examples:
    Perform a clean build with the default release type:
        $0 -n -cam /path/to/camera_sdk -imu /path/to/imu_sdk -lidar /path/to/lidar_sdk

    Build in debug mode:
        $0 -b debug -cam /path/to/camera_sdk -imu /path/to/imu_sdk -lidar /path/to/lidar_sdk

    Perform a clean build in debug mode:
        $0 -n -b debug -cam /path/to/camera_sdk -imu /path/to/imu_sdk -lidar /path/to/lidar_sdk
EOF
    exit 1
}

# Default options
BUILD_TYPE="release"
CAMERA_SDK_DIR=""
CAMERA_SDK_DIR_DEFAULT="./third_party/camera_sdk"
IMU_SDK_DIR=""
IMU_SDK_DIR_DEFAULT="./third_party/imu_sdk"
LIDAR_SDK_DIR=""
LIDAR_SDK_DIR_DEFAULT="./third_party/lidar_sdk"
CLEAN_BUILD=false

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -n|--newbuild)
            CLEAN_BUILD=true
            shift
            ;;
        -b|--buildtype)
            BUILD_TYPE=$2
            if [[ "$BUILD_TYPE" != "debug" && "$BUILD_TYPE" != "release" ]]; then
                echo "Error: Invalid build type. Use 'debug' or 'release'."
                usage
            fi
            shift 2
            ;;
        -cam|--camera_dir)
            CAMERA_SDK_DIR=$2
            shift 2
            ;;
        -imu|--imu_dir)
            IMU_SDK_DIR=$2
            shift 2
            ;;
        -lidar|--lidar_dir)
            LIDAR_SDK_DIR=$2
            shift 2
            ;;        
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

# Clear the current terminal
clear 

# SDK Directories
if [[ -z "$CAMERA_SDK_DIR" ]]; then
    if [[ -n "$CAMERA_SDK_DIR_DEFAULT" ]]; then
        echo "Using default camera SDK directory: ${CAMERA_SDK_DIR_DEFAULT}"
        CAMERA_SDK_DIR="${CAMERA_SDK_DIR_DEFAULT}"
    else
        echo "Error: Camera SDK directory is required."
        usage
    fi
fi
CAMERA_INCLUDE="${CAMERA_SDK_DIR}/include"
CAMERA_LIBRARY="${CAMERA_SDK_DIR}/lib"

if [[ -z "$IMU_SDK_DIR" ]]; then
    if [[ -n "$IMU_SDK_DIR_DEFAULT" ]]; then
        echo "Using default IMU SDK directory: ${IMU_SDK_DIR_DEFAULT}"
        IMU_SDK_DIR="${IMU_SDK_DIR_DEFAULT}"
    else
        echo "Error: IMU SDK directory is required."
        usage
    fi
fi
IMU_PLATFORM="${IMU_SDK_DIR}/platforms/linux"

if [[ -z "$LIDAR_SDK_DIR" ]]; then
    if [[ -n "$LIDAR_SDK_DIR_DEFAULT" ]]; then
        echo "Using default LiDAR SDK directory: ${LIDAR_SDK_DIR_DEFAULT}"
        LIDAR_SDK_DIR="${LIDAR_SDK_DIR_DEFAULT}"
    else
        echo "Error: LiDAR SDK directory is required."
        usage
    fi
fi
LIDAR_SCANNER="${LIDAR_SDK_DIR}/LidarScanner"

# Main logic
if $CLEAN_BUILD; then
    echo "Performing a clean build..."
    rm -rf build
    echo "Clean build completed."
fi

echo "Building project in $BUILD_TYPE mode..."
CMAKE_BUILD_TYPE="-DCMAKE_BUILD_TYPE=${BUILD_TYPE^}"
CMAKE_OPTIONS="-B build -S . \
    -DCMAKE_INSTALL_PREFIX=install ${CMAKE_BUILD_TYPE} \
    -DCAMERA_SDK_INCLUDE_DIR=${CAMERA_INCLUDE} \
    -DCAMERA_SDK_LIB_DIR=${CAMERA_LIBRARY} \
    -DINS_SDK=${IMU_SDK_DIR} \
    -DINS_PLATFORM=${IMU_PLATFORM} \
    -DLIDAR_SCANNER=${LIDAR_SCANNER}"

MAKE_COMMAND="cmake --build build --target install"

echo "Configuring project with CMake..."
cmake ${CMAKE_OPTIONS}

if [[ $? -eq 0 ]]; then
    echo "Building project..."
    eval ${MAKE_COMMAND}

    if [[ $? -eq 0 ]]; then
        echo "Build succeeded. Running executable..."
        if [[ "${BUILD_TYPE}" == "debug" ]]; then
            echo "Running in debug mode with GDB..."
            gdb --args ./install/bin/atis_proj_exec > debug_run.txt
        else
            echo "Running in release mode..."
            ./install/bin/atis_proj_exec > release_run.txt
        fi
    else
        echo "Build failed. Skipping execution."
        exit 1
    fi
else
    echo "CMake configuration failed."
    exit 1
fi
