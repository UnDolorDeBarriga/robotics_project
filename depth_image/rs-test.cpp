#include <librealsense2/rs.hpp>
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_frame.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <array>




#define MIN_DIST 0.01f  // Set the max distance (in meters) for the color gradient
#define MAX_DIST 5.0f  // Set the max distance (in meters) for the color gradient
#define WIDTH 640
#define HEIGHT 480
#define FPS 30

// Main function
int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Usage: %s <number of frames>\n", argv[0]);
        return EXIT_FAILURE;
    }
    int n_index = atoi(argv[1]);
    rs2_error* e = 0;

    // Create a context object
    rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);
    // Get a list of all the connected devices
    rs2_device_list* device_list = rs2_query_devices(ctx, &e);
    int dev_count = rs2_get_device_count(device_list, &e);
    printf("There are %d connected RealSense devices.\n", dev_count);
    if (0 == dev_count)
        return EXIT_FAILURE;
    // Get the first connected device
    rs2_device* dev = rs2_create_device(device_list, 0, &e);
    // Create a pipeline to configure, start, and stop camera streaming
    rs2_pipeline* pipeline = rs2_create_pipeline(ctx, &e);
    // Create a config instance, used to specify hardware configuration
    rs2_config* config = rs2_create_config(&e);
    // Request a specific configuration (depth stream)
    rs2_config_enable_stream(config, RS2_STREAM_DEPTH, 0, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS, &e);
    // Start the pipeline streaming
    rs2_pipeline_profile* pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);
    // Allocate buffer to store the depth image
    uint16_t* depth_data = new uint16_t[WIDTH * HEIGHT]();

    // Initialize OpenCV matrices to accumulate depth data
    cv::Mat accumulated_depth = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
    cv::Mat valid_pixel_count = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

    // Obtain the intrinsics
    rs2_stream_profile_list* stream_profiles = rs2_pipeline_profile_get_streams(pipeline_profile, &e);
    const rs2_stream_profile* depth_stream_profile = rs2_get_stream_profile(stream_profiles, 0, &e);
    rs2_intrinsics intrinsics;
    // Convert stream profile to video stream profile (depth stream is considered as a video stream)
    //const rs2_video_stream_profile* video_stream_profile = reinterpret_cast<const rs2_video_stream_profile*>(depth_stream_profile);
    rs2_get_video_stream_intrinsics(depth_stream_profile, &intrinsics, &e);



    // Clean up stream profiles
    rs2_delete_stream_profiles_list(stream_profiles);

    for (int frame_count = 0; frame_count < n_index; ++frame_count) {
        rs2_frame* frames = rs2_pipeline_wait_for_frames(pipeline, RS2_DEFAULT_TIMEOUT, &e);
        rs2_frame* depth_frame = rs2_extract_frame(frames, 0, &e);

        // Get depth data in 16-bit format
        const uint16_t* frame_data = (const uint16_t*)rs2_get_frame_data(depth_frame, &e);

        // Normalize depth values to max distance (in millimeters)
        int max_depth = MAX_DIST * 1000;  // Convert max distance to millimeters
        int min_depth = MIN_DIST * 1000;  // Convert max distance to millimeters
        for (int i = 0; i < WIDTH * HEIGHT; ++i) {
            uint16_t depth = frame_data[i];
            if (depth >= 0 && depth <= max_depth) {
                depth_data[i] = depth;
                valid_pixel_count.at<int>(i / WIDTH, i % WIDTH) += 1;
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
    }

    // Compute the mean depth image
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            if (valid_pixel_count.at<int>(y, x) > 0) {
                accumulated_depth.at<float>(y, x) /= valid_pixel_count.at<int>(y, x);
            }
        }
    }
    //cv::divide(accumulated_depth, valid_pixel_count, accumulated_depth);
    //accumulated_depth /= n_index;

    // Normalize and convert to 8-bit for visualization
    cv::Mat mean_depth_image;
    cv::normalize(accumulated_depth, mean_depth_image, 0, 255, cv::NORM_MINMAX);
    mean_depth_image.convertTo(mean_depth_image, CV_8UC1);

    // Apply color map
    cv::Mat depth_color;
    cv::applyColorMap(mean_depth_image, depth_color, cv::COLORMAP_JET);

    // Save the mean depth image
    char filename[50];
    sprintf(filename, "mean%d_depth_image.png", n_index);
    cv::imwrite(filename, depth_color);


    // Open a CSV file to write the depth data
    char filename2[50];
    sprintf(filename2, "mean%d_depth_csv.csv", n_index);
    std::ofstream csv_file(filename2);
    if (!csv_file.is_open()) {
        printf("Failed to open the CSV file.\n");
        return EXIT_FAILURE;
    }
    // Write the depth data to the CSV file
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            float depth_mm = accumulated_depth.at<float>(y, x);
            csv_file << depth_mm << ",";
        }
        csv_file << "\n";
    }
    // Close the CSV file
    csv_file.close();


    // Deproject pixels to 3D points
    std::vector<std::array<float, 3>> points;

    for (int i = 0; i < HEIGHT; ++i) {
        for (int j = 0; j < WIDTH; ++j) {
            float pixel[2] = { static_cast<float>(j), static_cast<float>(i) };
            float depth = accumulated_depth.at<float>(i, j);
            float point[3];
            rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth);
            points.push_back({ point[0], point[1], point[2] });
        }
    }

    // Optionally, save the 3D points to a file
    std::ofstream points_file("points.txt");
    for (const auto& point : points) {
        points_file << point[0] << "," << point[1] << "," << point[2] << "\n";
    }
    points_file.close();


    // Stop the pipeline
    rs2_pipeline_stop(pipeline, &e);

    // Release resources
    delete[] depth_data;
    rs2_delete_config(config);
    rs2_delete_pipeline(pipeline);
    rs2_delete_device(dev);
    rs2_delete_device_list(device_list);
    rs2_delete_context(ctx);
    return EXIT_SUCCESS;
}
