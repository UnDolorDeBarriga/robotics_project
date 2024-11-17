cmake_minimum_required(VERSION 3.10)

# Project name
project(MatrixProcessor)

# Find required packages
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${realsense2_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# Add executables
add_executable(matrix ../matrici.cpp ../spatial_transf.cpp)

# Link libraries
target_link_libraries(matrix ${realsense2_LIBRARY} ${OpenCV_LIBS} Eigen3::Eigen)