#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>

// Function to get the 3D coordinates of a specific pixel
rs2::vertex get_3d_point(const rs2::depth_frame& depth_frame, int x, int y) {
    // Get the depth scale (how depth values are mapped to meters)
    float depth_scale = depth_frame.get_units();

    // Get the intrinsic parameters of the depth frame
    auto intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    // Get the depth value (in pixels) from the selected pixel
    float depth_value = depth_frame.get_distance(x, y);

    // De-project the 2D pixel to 3D using depth and intrinsics
    float pixel_3d[3]; // Array to store the resulting 3D coordinates
    float pixel_2d[2] = {(float)x, (float)y};
    rs2_deproject_pixel_to_point(pixel_3d, &intrinsics, pixel_2d, depth_value);

    // Return the 3D point as a vertex
    return {pixel_3d[0], pixel_3d[1], pixel_3d[2]};
}

int main() {
    // Start RealSense pipeline
    rs2::pipeline pipe;
    pipe.start();

    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();

    // Get the depth frame
    auto depth = frames.get_depth_frame();

    // Specify the pixel coordinates
    int x = 320; // For example, center pixel in a 640x480 image
    int y = 240;

    // Get 3D coordinates of the pixel
    rs2::vertex point = get_3d_point(depth, x, y);

    // Print the 3D coordinates
    std::cout << "3D Coordinates of pixel (" << x << ", " << y << "): ";
    std::cout << "X: " << point.x << ", Y: " << point.y << ", Z: " << point.z << std::endl;

    return 0;
}
