# 3D Reconstruction with Intel RealSense Camera

## Overview

This project reconstructs 3D images using an Intel RealSense depth camera. The camera captures depth images and, with provided positional data, generates a point cloud representing the environment’s height variations. The system is designed for robotic applications, particularly in conjunction with ROS2, allowing a robot to map its surroundings and determine navigable areas.

## Installation and Usage

### Requirements

- Intel RealSense Camera
- ROS2 (if used in a robotic system)
- C++ Compiler (GCC/Clang)
- Python (for auxiliary scripts)
- OpenCV & librealsense

### Compilation

```bash
cd depth_image
mkdir build && cd build
cmake ..
make
```

### Running the Program

To execute the main processing pipeline, use the following format:

```bash
./main <number_of_images> <min_distance_mm> <max_distance_mm> <num_frames> <cell_discretization_mm>
```

Where:
- `<number_of_images>`: The number of images to be processed.
- `<min_distance_mm>`: The minimum distance threshold in millimeters.
- `<max_distance_mm>`: The maximum distance threshold in millimeters.
- `<num_frames>`: The number of frames to be averaged.
- `<cell_discretization_mm>`: The spatial resolution in millimeters.

## Future Improvements

- Integration with ROS2 nodes for real-time mapping.
- Optimization of depth-to-point cloud conversion.
- Improved noise filtering and calibration.

## Author

[Arnau Bayer Mena](https://github.com/UnDolorDeBarriga)


