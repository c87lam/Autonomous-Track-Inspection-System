cmake_minimum_required(VERSION 3.14)
project(atis_proj VERSION 1.0.0)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable stricter warnings for GCC and Clang
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
add_definitions(-DQT_NO_KEYWORDS)

# Dependencies
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)

# Eigen
include(FetchContent)
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen not found, downloading...")
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
    )
    FetchContent_MakeAvailable(eigen)
    set(Eigen3_DIR ${eigen_SOURCE_DIR})
endif()

# Qt5 Widgets
set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/qt5")
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Generic SDK directories
set(CAMERA_SDK_INCLUDE_DIR "" CACHE PATH "Path to Camera SDK include directory")
set(CAMERA_SDK_LIB_DIR "" CACHE PATH "Path to Camera SDK library directory")

if (NOT CAMERA_SDK_INCLUDE_DIR OR NOT CAMERA_SDK_LIB_DIR)
    message(FATAL_ERROR "Please specify both CAMERA_SDK_INCLUDE_DIR and CAMERA_SDK_LIB_DIR.")
endif()

set(IMU_SDK "" CACHE PATH "Path to IMU SDK folder")
set(IMU_PLATFORM "" CACHE PATH "Path to IMU linux platform")

if (NOT IMU_SDK OR NOT IMU_PLATFORM)
    message(FATAL_ERROR "Please specify the location of the IMU SDK.")
endif()

set(LIDAR_SDK_DIR "" CACHE PATH "Path to LiDAR SDK folder")
if (NOT LIDAR_SDK_DIR)
    message(FATAL_ERROR "Please specify the location of the LiDAR SDK.")
endif()

# Locate the Lidar Scanner library inside LiDAR SDK
find_library(LIDAR_SDK_LIBRARY NAMES LidarScanner PATHS ${LIDAR_SDK_DIR})
if (NOT LIDAR_SDK_LIBRARY)
    message(FATAL_ERROR "Could not find LidarScanner shared library (.so) in ${LIDAR_SDK_DIR}")
endif()

# Locate headers inside LIDAR SDK
if (EXISTS "${LIDAR_SDK_DIR}/include")
    set(LIDAR_SDK_INCLUDE "${LIDAR_SDK_DIR}/include")
else()
    set(LIDAR_SDK_INCLUDE "${LIDAR_SDK_DIR}")
endif()

# Source and header files
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS "src/*.cpp"
    "${IMU_SDK}/Driver.cpp"
    "${IMU_SDK}/Parser.cpp"
    "${IMU_PLATFORM}/Client.cpp"
    "${IMU_PLATFORM}/SerialPort.cpp")

file(GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS "include/*.hpp")

# Build executable
add_executable(atis_proj_exec ${HEADER_FILES} ${SRC_FILES})

# Include directories and link libraries
target_include_directories(atis_proj_exec PRIVATE
    include
    ${TBB_INCLUDE_DIRS}
    ${CAMERA_SDK_INCLUDE_DIR}
    ${IMU_SDK}
    ${IMU_PLATFORM}
    ${LIDAR_SDK_INCLUDE}
    ${Eigen3_DIR}
)

# Link Camera SDK libraries
target_link_directories(atis_proj_exec PRIVATE
    ${CAMERA_SDK_LIB_DIR}
)

# Link libraries
target_link_libraries(atis_proj_exec PRIVATE
    TBB::tbb
    yaml-cpp
    Threads::Threads
    ${OpenCV_LIBS}
    ${LIDAR_SDK_LIBRARY}
    Qt5::Widgets
    spdlog::spdlog
)

set_target_properties(atis_proj_exec PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_RPATH "${LIDAR_SDK_DIR};${CAMERA_SDK_LIB_DIR}"
    INSTALL_RPATH "${LIDAR_SDK_DIR};${CAMERA_SDK_LIB_DIR}"
)

# Compiler options
target_compile_options(atis_proj_exec PRIVATE
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Release>:-O3 -DNDEBUG -march=native>
)

# Install
install(TARGETS atis_proj_exec RUNTIME DESTINATION bin)

message("[INFO] Compilation setup complete.")
