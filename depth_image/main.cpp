#include "resources.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <chrono>
#include <thread>

// Main function
int main(int argc, char *argv[]) {
    if (argc != 4) {
        printf("Usage: %s <maximum distance(m)> <number of frames> <cell discretization(mm)>\n", argv[0]);
        return EXIT_FAILURE;
    }
    int max_dist = atoi(argv[1]);
    int n_index = atoi(argv[2]);
    int cell_dim = atoi(argv[3]);
    int max_depth = max_dist * 1000;  // Convert max distance to millimeters

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

    // Get depth intrinsics
    rs2_intrinsics intrinsics;
    
    for (int image_n = 0; image_n < N_IMAGES; image_n++) {
        // Reinitialize OpenCV matrices to accumulate depth data
        Mat accumulated_depth = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
        Mat valid_pixel_count = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
        
        for (int frame_count = 0; frame_count < n_index; ++frame_count) {
            // Wait for the next set of frames
            frameset frames = pipeline.wait_for_frames();
            depth_frame depth_frame = frames.get_depth_frame();
            intrinsics = depth_frame.get_profile().as<video_stream_profile>().get_intrinsics();

            Mat depth_data = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

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
        sprintf(o_filename, "../data/reference_points_image%d.txt", image_n);
    
        // Compute the mean depth image
        get_mean_depth(accumulated_depth, valid_pixel_count);
        
        // Write the depth data to a CSV file
        write_depth_to_csv(accumulated_depth, n_index, image_n);
        
        // Deproject the mean depth image into 3D points
        auto points_image = deproject_depth_to_3d(i_filename, accumulated_depth, intrinsics, image_n);

        // Write the mean depth image to a PNG file
        write_depth_to_image(accumulated_depth, max_depth, n_index, image_n);
        
        // Get the user points for the camera position and angle
        Vector3f camera_position;
        Vector3f camera_angle;
        get_user_points(image_n, camera_position, camera_angle);

        Matrix4d M = create_transformation_matrix(camera_position, camera_angle);

        transformate_cordinates(i_filename, o_filename, M, maxAbsX, maxAbsY);


        // Wait for a keyboard input
        if (image_n != N_IMAGES-1) {
            printf("Image %d done.\nPress any key to continue...\n", image_n);
            getchar();
        } else {
            printf("Image %d done.\n", image_n);
        }
    }
    MatrixXd big_ass_matrix;
    big_ass_matrix = createMatrix(maxAbsX, maxAbsY, cell_dim, center_y, center_x);


    // Stop the pipeline
    pipeline.stop();
    return 0;
}
