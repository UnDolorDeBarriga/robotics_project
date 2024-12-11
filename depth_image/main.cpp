#include "resources.h"
// #include <stdlib.h>
// #include <stdint.h>
// #include <stdio.h>
// #include <chrono>
// #include <thread>


// Main function
int main(int argc, char *argv[]) {
    if (argc != 6) {
        printf("Usage: %s <number of images that are going to be computed> <minimum distance(m)> <maximum distance(m)> <number of frames> <cell discretization(mm)>\n", argv[0]);
        return EXIT_FAILURE;
    }
    int n_images = atoi(argv[1]);
    int min_dist = atoi(argv[2]);
    int max_dist = atoi(argv[3]);
    int n_index = atoi(argv[4]);
    int cell_dim = atoi(argv[5]);

    double maxAbsX=0;
    double maxAbsY=0;

    Vector3f o_camera_position, o_camera_angle = Vector3f::Zero();

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
    } else {
        #if DEBUG
        printf("Device found.\n");
        #endif
    }

    // Get depth intrinsics
    rs2_intrinsics intrinsics;
    

    // TODO:
        // Camera points Combinated image 1 header -> Position of the camera
        // Camera points Combinated image n header -> Points of the camera in n
        // Camera points Combinated big -> image 1 and calibration data
            // calib 1 x [1088, 1938, 2701, 3940, 4925, 6225, 7323, 7846, 8704, 9811]
            // calib 1 y [15,   28,   43,   143,  183,  250,  307,  375,  171,  300]
            // calib 2 x [1098, 2054, 2892, 3894, 5002, 6178, 6899, 8159, 9052, 9827]
            // calib 2 y [23,   38,   79,   53,   74,   91,   137,  265,  215,  253]

    
    for (int image_n = 0; image_n < n_images; image_n++) {
        // Reinitialize OpenCV matrices to accumulate depth data
        Mat accumulated_depth = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
        Mat valid_pixel_count = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
        
        intrinsics = get_main_frames_count(pipeline, n_index, accumulated_depth, valid_pixel_count, min_dist, max_dist);
        //aqui va too       

        char i_filename[100];
        sprintf(i_filename, "../data/camera_points_image%d.txt", image_n);
        char o_filename[100];
        sprintf(o_filename, "../data/reference_points_image%d.txt", image_n);
        filenames.push_back(o_filename);
        char pos_filename[100];
        sprintf(pos_filename, "../position_camera.txt");
        Vector3f camera_position, camera_angle = Vector3f::Zero();
        write_data_to_files(n_index, image_n, i_filename, o_filename, pos_filename, accumulated_depth, valid_pixel_count, intrinsics,min_dist, max_dist, maxAbsX, maxAbsY, camera_position, camera_angle);
        if (image_n == 0) {
            o_camera_position = camera_position;
            o_camera_angle = camera_angle;
        }
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
    
    int num_rows = (ceil((maxAbsY) / cell_dim))+1;
    int num_cols = (ceil((2 * maxAbsX) / cell_dim))+1;
    
    center_y = num_rows;
    center_x = num_cols / 2;

    int e = 20;

    Mat big_matrix_combined = Mat::zeros(num_rows, num_cols, CV_8UC1);
    Mat matrix_to_be_merged = Mat::zeros(num_rows, num_cols, CV_8UC1);

    cout << "Num Cols: " << num_cols << endl;
    cout << "Num Rows: " << num_rows << endl;

    populate_matrix_from_file(filenames[0].c_str(), big_matrix_combined, center_y, center_x, cell_dim, num_rows, num_cols);
    populate_matrix_from_file(filenames[1].c_str(), matrix_to_be_merged, center_y, center_x, cell_dim, num_rows, num_cols);

    Mat big_matrix_combined1_photo = big_matrix_combined.clone();
    int max = save_matrix_with_zeros(big_matrix_combined1_photo, "../data/combinated_info_points_1.txt", num_rows, num_cols);
    cv::imwrite("../data/big_matrix_image1.png", big_matrix_combined1_photo);

    Mat big_matrix_combined2_photo = matrix_to_be_merged.clone();
    max = save_matrix_with_zeros(big_matrix_combined2_photo, "../data/combinated_info_points_2.txt", num_rows, num_cols);
    cv::imwrite("../data/big_matrix_image2.png", big_matrix_combined2_photo);

    if(check_matrix(big_matrix_combined, matrix_to_be_merged, num_rows, num_cols, e)){
        populate_matrix_from_file(filenames[1].c_str(), big_matrix_combined, center_y, center_x, cell_dim, num_rows, num_cols);
        cout << "Images merged" << endl;
    }
    else{
        cout << "Images too diferent to be merged" << endl;
    }
    

    max = save_matrix_with_zeros(big_matrix_combined, "../data/combinated_info_points.txt", num_rows, num_cols);
   
    cv::imwrite("../data/big_matrix_image.png", big_matrix_combined);

    return 0;

    // Eigen::SparseMatrix<double> big_ass_sparse_matrix_combined;
    // big_ass_sparse_matrix_combined.resize(ceil((2 * maxAbsY) / cell_dim) + 1, ceil((2 * maxAbsX) / cell_dim) + 1);
    // printf("2");

    // for (const auto& filename : filenames) {
    //     populate_sparse_matrix_from_file(filename.c_str(), big_ass_sparse_matrix_combined, center_y, center_x, cell_dim);
    // }

    return 0;
}
