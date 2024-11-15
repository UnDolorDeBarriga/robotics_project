#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <array>
#include <vector>
#include <Eigen/Dense>

#define MIN_DIST 0.01f  // Set the max distance (in meters) for the color gradient
//#define MAX_DIST 15.0f  // Set the max distance (in meters) for the color gradient
#define WIDTH 640
#define HEIGHT 480
#define FPS 30


// Function to deproject depth frame into 3D points
std::vector<Eigen::Vector3f> deproject_depth_to_3d(cv::Mat accumulated_depth, rs2_intrinsics intrinsics, int image_n) {
    std::vector<Eigen::Vector3f> points;

    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            float depth = accumulated_depth.at<float>(y, x);
            if (depth > 0) {  // Ignore invalid depth
                float point[3];
                float pixel[2] = { static_cast<float>(x), static_cast<float>(y) };
                rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth);
                points.emplace_back(point[0], point[1], point[2]);
            }
        }
    }
    // Optionally, save the 3D points to a file
    char filename[50];
    sprintf(filename, "../data/points_image%d.txt", image_n);
    std::ofstream points_file(filename);
    for (const auto& point : points) {
        points_file << point[0] << "," << point[1] << "," << point[2] << "\n";
    }
    points_file.close();
    return points;
}

// Main function
int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: %s <maximum distance> <number of frames>\n", argv[0]);
        return EXIT_FAILURE;
    }
    int max_dist = atoi(argv[1]);
    int n_index = atoi(argv[2]);
    rs2_error* e = 0;

    // Initialize RealSense pipeline
    rs2::pipeline pipeline;
    rs2::config config;
    pipeline.start(config);
    rs2::device dev = pipeline.get_active_profile().get_device();
    if (!dev) {
        printf("No device found.\n");
        return EXIT_FAILURE;
    }
    else {
        printf("Device found.\n");
    }
    config.enable_stream(RS2_STREAM_DEPTH, WIDTH, WIDTH, RS2_FORMAT_Z16, FPS);
    

    // Get depth intrinsics
    rs2_intrinsics intrinsics;



    for (int image_n = 0; image_n < 2; image_n++) {
        // Initialize OpenCV matrices to accumulate depth data
        cv::Mat accumulated_depth = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
        cv::Mat valid_pixel_count = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
        
        for (int frame_count = 0; frame_count < n_index; ++frame_count) {
            // Wait for the next set of frames
            rs2::frameset frames = pipeline.wait_for_frames();
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            intrinsics = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
            

            cv::Mat depth_data = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

            // Normalize depth values to max distance (in millimeters)
            int max_depth = max_dist * 1000;  // Convert max distance to millimeters
            int min_depth = MIN_DIST * 1000;  // Convert max distance to millimeters
            for (int i = 0; i < WIDTH; i++){
                for (int j = 0; j < HEIGHT; j++){
                    float depth = depth_frame.get_distance(i, j);
                    if (depth >= 0) {
                        if (depth >= max_depth) {
                            depth_data.at<float>(j, i) = max_depth*1000;
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
            cv::Mat depth_float;
            accumulated_depth += depth_data;

        }

        // Compute the mean depth image
        for (int y = 0; y < HEIGHT; ++y) {
            for (int x = 0; x < WIDTH; ++x) {
                if (valid_pixel_count.at<int>(y, x) > 0) {
                    accumulated_depth.at<float>(y, x) /= valid_pixel_count.at<int>(y, x);
                }
            }
        }
        // Normalize and convert to 8-bit for visualization
        cv::Mat mean_depth_image;
        cv::normalize(accumulated_depth, mean_depth_image, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
        // Normalize with a fixed input range of (min_depth, max_depth) to an output range (0, 255)
        mean_depth_image.convertTo(mean_depth_image, CV_8UC1);

        // Apply color map
        cv::Mat depth_color;
        cv::applyColorMap(mean_depth_image, depth_color, cv::COLORMAP_JET);

        // Save the mean depth image
        char filename[50];
        sprintf(filename, "../data/mean%d_depth_image%d.png", n_index, image_n);
        cv::imwrite(filename, depth_color);


        // Open a CSV file to write the depth data
        sprintf(filename, "../data/mean%d_depth%d.csv", n_index, image_n);
        std::ofstream csv_file(filename);
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

        // Deproject the mean depth image into 3D points
        auto points_image = deproject_depth_to_3d(accumulated_depth, intrinsics, image_n);

        // Wait for a keyboard input
        if (image_n == 0) {
            printf("Image 0 done.\nPress any key to continue...\n");
            getchar();
        }
        else {
            printf("Image 1 done.\n");
        }
    }
    // Stop the pipeline
    pipeline.stop();
    return 0;
}
