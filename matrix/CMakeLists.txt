cmake_minimum_required(VERSION 3.10)

# Project name
project(MatrixProcessor)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# Add executables
add_executable(matrix ../matrici.cpp ../spatial_transf.cpp)
add_executable(better ../matrici_better.cpp ../spatial_transf.cpp)

# Link libraries
target_link_libraries(matrix ${OpenCV_LIBS} Eigen3::Eigen)
target_link_libraries(better ${OpenCV_LIBS} Eigen3::Eigen)


# Custom targets for individual builds
add_custom_target(build_matrix DEPENDS matrix)
add_custom_target(build_better DEPENDS better)