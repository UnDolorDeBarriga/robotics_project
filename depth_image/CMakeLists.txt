cmake_minimum_required(VERSION 3.10)

# Project name
project(DepthImageProcessing)

# Find required packages
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${realsense2_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# Add executables
add_executable(rs-test ../rs-test.cpp)
add_executable(matrix ../matrix.cpp)

# Link libraries
target_link_libraries(rs-test ${realsense2_LIBRARY} ${OpenCV_LIBS})
target_link_libraries(matrix ${realsense2_LIBRARY} ${OpelsnCV_LIBS} Eigen3::Eigen)

# g++ -o rs-test rs-test.cpp -I/usr/local/include -L/usr/local/lib -lrealsense2 `pkg-config --cflags --libs opencv4`