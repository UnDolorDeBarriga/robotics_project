cmake_minimum_required(VERSION 3.10)

# Project name
project(DepthImageProcessing)

# Find required packages
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# Include directories
include_directories(${realsense2_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# Add executables
add_executable(main ../main.cpp ../resources.cpp)

# Link libraries
target_link_libraries(main ${realsense2_LIBRARY} ${OpenCV_LIBS} ${EIGEN3_LIBRARIES} ${OPENGL_LIBRARIES} glfw)

# g++ -o rs-test rs-test.cpp -I/usr/local/include -L/usr/local/lib -lrealsense2 `pkg-config --cflags --libs opencv4`