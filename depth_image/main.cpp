#include "resources.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <chrono>
#include <thread>

// Main function
int main(int argc, char *argv[]) {
    if (argc != 5) {
        printf("Usage: %s <number of images that are going to be computed> <maximum distance(m)> <number of frames> <cell discretization(mm)>\n", argv[0]);
        return EXIT_FAILURE;
    }
    int n_images = atoi(argv[1]);
    int max_dist = atoi(argv[2]);
    int n_index = atoi(argv[3]);
    int cell_dim = atoi(argv[4]);
    float max_depth = max_dist * 1000;  // Convert max distance to millimeters

    double maxAbsX=0;
    double maxAbsY=0;

    int center_x;
    int center_y;

    vector<string> filenames;

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
    
    for (int image_n = 0; image_n < n_images; image_n++) {
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
                    if (depth >= 1) {
                        if (depth >= max_dist) {
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

        printf("Accumulated depth data for image %d\n", image_n);

        char i_filename[100];
        sprintf(i_filename, "../data/camera_points_image%d.txt", image_n);
        char o_filename[100];
        sprintf(o_filename, "../data/reference_points_image%d.txt", image_n);
        filenames.push_back(o_filename);
    
        // Compute the mean depth image
        Mat average_depth = get_mean_depth(accumulated_depth, valid_pixel_count, max_depth);
        printf("Computed mean depth for image %d\n", image_n);
        
        // Write the depth data to a CSV file
        write_depth_to_csv(average_depth, n_index, image_n);
        printf("Wrote depth data to CSV for image %d\n", image_n);
        
        // Deproject the mean depth image into 3D points
        auto points_image = deproject_depth_to_3d(i_filename, average_depth, intrinsics, image_n);
        printf("Deprojected depth to 3D for image %d\n", image_n);

        // Write the mean depth image to a PNG file
        write_depth_to_image(average_depth, max_depth, n_index, image_n);
        printf("Wrote depth image to PNG for image %d\n", image_n);
        
        // Get the user points for the camera position and angle
        Vector3f camera_position;
        Vector3f camera_angle;
        
        //TODO: Read the user points from a file every time
        //get_user_points(image_n, camera_position, camera_angle);
        camera_position = Vector3f(0.0, 0.0, 0.0);
        camera_angle = Vector3f(-90.0, 0.0, 0.0);

        Matrix4d M = create_transformation_matrix(camera_position, camera_angle);
        printf("Created transformation matrix for image %d\n", image_n);

        transformate_cordinates(i_filename, o_filename, M, maxAbsX, maxAbsY, camera_position, camera_angle);
        printf("Transformed coordinates for image %d\n", image_n);

        // Wait for a keyboard input
        if (image_n != n_images-1) {
            printf("Image %d done.\nPress any key to continue...\n", image_n);
            getchar();
        } else {
            printf("Image %d done.\n", image_n);
        }
    }
    // Stop the pipeline
    pipeline.stop();
    
    printf("1");
    
    MatrixXd big_ass_matrix_combined = create_matrix(maxAbsX, maxAbsY, cell_dim, center_y, center_x);
    printf("2");

    return 0;

    printf("3");

    printf("N Rows: %d\n", big_ass_matrix_combined.rows());
    printf("N Cols: %d\n", big_ass_matrix_combined.cols());


    return 0;
}
