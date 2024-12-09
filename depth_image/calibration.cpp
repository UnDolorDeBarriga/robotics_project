#include "resources.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <fstream>

// Main function
int main(int argc, char *argv[]) {
    if (argc != 4) {
        printf("Usage: %s <maximum distance(m)> <number of frames to average> <dist to calculate mm>\n", argv[0]);
        return EXIT_FAILURE;
    }
    int max_dist = atoi(argv[1]);
    int n_index = atoi(argv[2]);
    int dist = atoi(argv[3]);
    int dist_cm = round(dist/10);

    double maxAbsX=0;
    double maxAbsY=0;

    int center_x;
    int center_y;

    // Initialize RealSense pipeline
    pipeline pipeline;
    config config;
    config.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
    pipeline.start(config);
    device dev = pipeline.get_active_profile().get_device();
    if (!dev) {
        printf("No device found.\n");
        return EXIT_FAILURE;
    }
    else {
        printf("Device found.\n");
    }
    
    // Reinitialize OpenCV matrices to accumulate depth data
    Mat accumulated_depth = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
    Mat valid_pixel_count = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
    

    for (int frame_count = 0; frame_count < n_index; ++frame_count) {
        // Wait for the next set of frames
        frameset frames = pipeline.wait_for_frames();
        depth_frame depth_frame = frames.get_depth_frame();

        Mat depth_data = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
        // Normalize depth values to max distance (in millimeters)
        for (int i = 0; i < WIDTH; i++){
            for (int j = 0; j < HEIGHT; j++){
                float depth = depth_frame.get_distance(i, j);
                if (depth >= 0) {
                    if (depth >= max_dist) {
                        depth_data.at<float>(j, i) = (max_dist*1000);
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

    // Compute the mean depth image
    Mat average_depth = get_mean_depth(accumulated_depth, valid_pixel_count, max_dist);
   
    int square_x_m = WIDTH/2;
    int square_y_m = HEIGHT/2;

    // Write the depth data to a CSV file
    char filename[50];
    sprintf(filename, "../data_calibration/mean_depth_%dcm.csv", dist_cm);
    ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        printf("Failed to open the CSV file.\n");
        return 0;
    }
    for(int y = 0 ; y < HEIGHT ; y++) {
        for(int x = 0 ; x < WIDTH ; x++) {
            float depth_mm = average_depth.at<float>(y, x);
            csv_file << depth_mm << ",";
        }
        csv_file << "\n";

    }
    csv_file.close();
    


    
    int average_depth_center = 0;
    int standard_deviation = 0;
    int n_pixels = 0;

    for(int i = (square_x_m-50); i < (square_x_m+50) ; i++) {
        for(int j = (square_y_m-50); j < (square_y_m+50) ; j++) {
            average_depth_center += average_depth.at<float>(j, i);
            n_pixels++;
        }
    }
    average_depth_center /= n_pixels;

    for(int i = (square_x_m-50); i < (square_x_m+50) ; i++) {
        for(int j = (square_y_m-50); j < (square_y_m+50) ; j++) {
            standard_deviation += pow(average_depth.at<float>(j, i) - average_depth_center, 2);
        }
    }
    standard_deviation = sqrt(standard_deviation/n_pixels);

    printf("Average depth in the center of the image: %d mm\n", average_depth_center);
    printf("Standard deviation of the depth in the center of the image: %d mm\n", standard_deviation);
    printf("Number of pixels: %d\n", n_pixels);

    // Write the mean depth image to a PNG file
    Mat mean_depth_image;
    for(int i = 0; i < WIDTH; i++) {
        for(int j = 0; j < HEIGHT; j++) {
            if((i == (square_x_m-50) && (j <= (square_y_m+50) && j >= (square_y_m-50))) || (i == (square_x_m+50) && (j <= (square_y_m+50) && j >= (square_y_m-50))) || (j == (square_y_m-50) && (i <= (square_x_m+50) && i >= (square_x_m-50))) || (j == (square_y_m+50) && (i <= (square_x_m+50) && i >= (square_x_m-50))) ) {
                average_depth.at<float>(j, i) = max_dist*1000;
            }
        }
    }
    average_depth.convertTo(mean_depth_image, CV_8UC1, 255.0 / (max_dist*1000), -255.0 / (max_dist*1000));
    Mat depth_color;
    applyColorMap(mean_depth_image, depth_color, COLORMAP_JET);
    sprintf(filename, "../data_calibration/mean_depth_%dcm.png",dist_cm);
    imwrite(filename, depth_color);

    // Save the mean and standard deviation to a file
    sprintf(filename, "../data_calibration/params_calibration.txt");
    ofstream stats_file(filename, ios::app); // Open file in append mode
    if (!stats_file.is_open()) {
        printf("Failed to open the stats file.\n");
        return 0;
    }
    stats_file << "Dist: " << dist_cm << " cm, Averaged in " <<  n_index << " frames\n";
    stats_file << "Average: " << average_depth_center << " mm\n";
    stats_file << "Stdd: " << standard_deviation << " mm\n";
    stats_file << endl;
    stats_file.close();




    // Stop the pipeline
    pipeline.stop();
    
    return 0;
}
