#include <librealsense2/rs.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_frame.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#define MAX_DIST 15.0f  // Set the max distance (in meters) for the color gradient
#define WIDTH 640
#define HEIGHT 480
#define FPS 30

// Helper function to handle errors
void check_error(rs2_error* e) {
    if (e) {
        const char* err_msg = rs2_get_error_message(e);
        printf("RealSense error: %s\n", err_msg);
        exit(EXIT_FAILURE);
    }
}

// Function to determine the maximum depth value in the frame
float get_max_depth_value(const uint16_t* frame_data, int width, int height) {
    float max_depth = 0.0f;
    for (int i = 0; i < width * height; ++i) {
        uint16_t depth = frame_data[i];
        if (depth > max_depth) {
            max_depth = depth;
        }
    }
    return max_depth / 1000.0f;  // Convert from mm to meters
}

// Main function
int main(int argc, char *argv[]) {
    rs2_error* e = 0;

    // Create a context object
    rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);
    check_error(e);

    // Get a list of all the connected devices
    rs2_device_list* device_list = rs2_query_devices(ctx, &e);
    check_error(e);

    int dev_count = rs2_get_device_count(device_list, &e);
    check_error(e);
    printf("There are %d connected RealSense devices.\n", dev_count);
    if (0 == dev_count)
        return EXIT_FAILURE;

    // Get the first connected device
    rs2_device* dev = rs2_create_device(device_list, 0, &e);
    check_error(e);

    // Create a pipeline to configure, start, and stop camera streaming
    rs2_pipeline* pipeline = rs2_create_pipeline(ctx, &e);
    check_error(e);

    // Create a config instance, used to specify hardware configuration
    rs2_config* config = rs2_create_config(&e);
    check_error(e);

    // Request a specific configuration (depth stream)
    rs2_config_enable_stream(config, RS2_STREAM_DEPTH, 0, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS, &e);
    check_error(e);

    // Start the pipeline streaming
    rs2_pipeline_profile* pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);
    check_error(e);

    // Allocate buffer to store the depth image
    uint16_t* depth_data = (uint16_t*)calloc(WIDTH * HEIGHT, sizeof(uint16_t));

    // Wait for the first frame
    rs2_frame* frames = rs2_pipeline_wait_for_frames(pipeline, RS2_DEFAULT_TIMEOUT, &e);
    check_error(e);

    // Extract depth frame
    rs2_frame* depth_frame = rs2_extract_frame(frames, 0, &e);
    check_error(e);

    // Get depth data in 16-bit format
    const uint16_t* frame_data = (const uint16_t*)rs2_get_frame_data(depth_frame, &e);
    check_error(e);

    // Normalize depth values to max distance (in millimeters)
    int max_depth = MAX_DIST * 1000;  // Convert max distance to millimeters
    for (int i = 0; i < WIDTH * HEIGHT; ++i) {
        uint16_t depth = frame_data[i];
        if (depth > 0 && depth < max_depth) {
            depth_data[i] = depth;
        } else {
            depth_data[i] = 0;  // Set out-of-range depth to 0
        }
    }

    // Convert depth to color gradient (using OpenCV)
    cv::Mat depth_image(HEIGHT, WIDTH, CV_16UC1, depth_data);

    // Apply color map to depth image
    cv::Mat depth_color;
    cv::normalize(depth_image, depth_image, 0, 255, cv::NORM_MINMAX);  // Normalize depth to 0-255 range
    depth_image.convertTo(depth_color, CV_8UC1);  // Convert to 8-bit for visualization
    cv::applyColorMap(depth_color, depth_color, cv::COLORMAP_JET);  // Apply color map

    // Save the depth image as PNG
    cv::imwrite("depth_image.png", depth_color);

    // Release resources
    rs2_release_frame(depth_frame);
    rs2_release_frame(frames);

    // Stop the pipeline
    rs2_pipeline_stop(pipeline, &e);
    check_error(e);

    // Release resources
    free(depth_data);
    rs2_delete_config(config);
    rs2_delete_pipeline(pipeline);
    rs2_delete_device(dev);
    rs2_delete_device_list(device_list);
    rs2_delete_context(ctx);

    printf("Depth image saved as 'depth_image.png'.\n");

    return EXIT_SUCCESS;
}
