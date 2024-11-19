#include "resources.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <chrono>
#include <thread>

// Main function
int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: %s <maximum distance> <number of frames>\n", argv[0]);
        return EXIT_FAILURE;
    }
    int max_dist = atoi(argv[1]);
    int n_index = atoi(argv[2]);
    int max_depth = max_dist * 1000;  // Convert max distance to millimeters

    // Initialize RealSense pipeline
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
    pipeline.start(config);
    rs2::device dev = pipeline.get_active_profile().get_device();
    if (!dev) {
        printf("No device found.\n");
        return EXIT_FAILURE;
    }
    else {
        printf("Device found.\n");
    }

    // Get depth intrinsics
    rs2_intrinsics intrinsics;
    
    for (int image_n = 0; image_n < N_IMAGES; image_n++) {
        // Reinitialize OpenCV matrices to accumulate depth data
        cv::Mat accumulated_depth = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
        cv::Mat valid_pixel_count = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
        
        for (int frame_count = 0; frame_count < n_index; ++frame_count) {
            // Wait for the next set of frames
            rs2::frameset frames = pipeline.wait_for_frames();
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

            cv::Mat depth_data = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

            // Normalize depth values to max distance (in millimeters)
            for (int i = 0; i < WIDTH; i++){
                for (int j = 0; j < HEIGHT; j++){
                    float depth = depth_frame.get_distance(i, j);
                    if (depth >= 0) {
                        if (depth >= max_depth) {
                            depth_data.at<float>(j, i) = max_depth;
                        }
                        else {
                            depth_data.at<float>(j, i) = depth * 1000;  // Convert depth to millimeters
                        }
                        valid_pixel_count.at<int>(j, i) += 1;
                    }
                    else {
                        depth_data.at<float>(j, i) = 0;  // Set out-of-range depth to 0
                    }
                }
            }
            // Accumulate depth data
            accumulated_depth += depth_data;
        }

        char i_filename[100];
        sprintf(i_filename, "../data/camera_points_image%d.txt", image_n);
        char o_filename[100];
        sprintf(i_filename, "../data/reference_points_image%d.txt", image_n);
    
        // Compute the mean depth image
        get_mean_depth(accumulated_depth, valid_pixel_count);
        
        // Write the depth data to a CSV file
        write_depth_to_csv(accumulated_depth, n_index, image_n);
        
        // Deproject the mean depth image into 3D points
        auto points_image = deproject_depth_to_3d(i_filename, accumulated_depth, intrinsics, image_n);

        // Write the mean depth image to a PNG file
        write_depth_to_image(accumulated_depth, max_depth, n_index, image_n);
        
        // Get the user points for the camera position and angle
        Eigen::Vector3f camera_position;
        Eigen::Vector3f camera_angle;
        //get_user_points(image_n, camera_position, camera_angle);

        // Wait for a keyboard input
        if (image_n != N_IMAGES-1) {
            printf("Image %d done.\nPress any key to continue...\n", image_n);
            getchar();
        } else {
            printf("Image %d done.\n", image_n);
        }
    }


    // Stop the pipeline
    pipeline.stop();
    return 0;
}
