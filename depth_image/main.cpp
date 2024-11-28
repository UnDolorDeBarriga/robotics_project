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
        
        intrinsics = get_main_frames_count(pipeline, n_index, accumulated_depth, valid_pixel_count, max_dist);
        //aqui va too       

        char i_filename[100];
        sprintf(i_filename, "../data/camera_points_image%d.txt", image_n);
        char o_filename[100];
        sprintf(o_filename, "../data/reference_points_image%d.txt", image_n);
        filenames.push_back(o_filename);

        write_data_to_files(n_index, image_n, i_filename, o_filename, accumulated_depth, valid_pixel_count, intrinsics, max_dist, maxAbsX, maxAbsY);

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
