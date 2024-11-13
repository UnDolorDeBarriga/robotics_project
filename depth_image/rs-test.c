#include <librealsense2/rs.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_frame.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#define MIN_DIST 0.1f  // Set the max distance (in meters) for the color gradient
#define MAX_DIST 5.0f  // Set the max distance (in meters) for the color gradient
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

    // Initialize OpenCV matrices to accumulate depth data
    cv::Mat accumulated_depth = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

    for (int frame_count = 0; frame_count < 10; ++frame_count) {
        // Wait for new frames
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
        int min_depth = MIN_DIST * 1000;  // Convert max distance to millimeters
        for (int i = 0; i < WIDTH * HEIGHT; ++i) {
            uint16_t depth = frame_data[i];
            if (depth >= min_depth && depth <= max_depth) {
                depth_data[i] = depth;
            } else {
                depth_data[i] = 0;  // Set out-of-range depth to 0
            }
        }

        // Convert depth to OpenCV matrix
        cv::Mat depth_image(HEIGHT, WIDTH, CV_16UC1, depth_data);

        // Accumulate depth data
        cv::Mat depth_float;
        depth_image.convertTo(depth_float, CV_32FC1);
        accumulated_depth += depth_float;

        // Release the frame
        rs2_release_frame(depth_frame);
        rs2_release_frame(frames);
    }

    // Compute the mean depth image
    accumulated_depth /= 10;

    // Normalize and convert to 8-bit for visualization
    cv::Mat mean_depth_image;
    cv::normalize(accumulated_depth, mean_depth_image, 0, 255, cv::NORM_MINMAX);
    mean_depth_image.convertTo(mean_depth_image, CV_8UC1);

    // Apply color map
    cv::Mat depth_color;
    cv::applyColorMap(mean_depth_image, depth_color, cv::COLORMAP_JET);

    // Save the mean depth image
    cv::imwrite("mean10_depth_image.png", depth_color);

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

    return EXIT_SUCCESS;
}
