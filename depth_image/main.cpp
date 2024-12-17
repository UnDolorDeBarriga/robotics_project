#include "resources.h"
// #include <stdlib.h>
// #include <stdint.h>
// #include <stdio.h>
// #include <chrono>
// #include <thread>


// Main function
int main(int argc, char *argv[]) {
    if (argc != 6) {
        printf("Usage: %s <number of images that are going to be computed> <minimum distance(mm)> <maximum distance(mm)> <number of frames> <cell discretization(mm)>\n", argv[0]);
        return EXIT_FAILURE;
    }
    int n_images = atoi(argv[1]);
    int min_dist = atoi(argv[2]);
    int max_dist = atoi(argv[3]);
    int n_index = atoi(argv[4]);
    int cell_dim = atoi(argv[5]);

    double maxAbsX=0;
    double maxAbsY=0;

    int center_x;
    int center_y;

    system("rm ../data/*");

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
        write_data_to_files(n_index, image_n, i_filename, o_filename, pos_filename, accumulated_depth, valid_pixel_count, intrinsics, min_dist, max_dist, maxAbsX, maxAbsY);
        
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
    cout << "Num Cols: " << num_cols << endl;
    cout << "Num Rows: " << num_rows << endl;

    Vector3f o_camera_position;
    Mat big_matrix_combined = Mat::zeros(num_rows, num_cols, CV_32SC1);
    o_camera_position = populate_matrix_from_file(filenames[0].c_str(), big_matrix_combined, center_y, center_x, cell_dim, num_rows, num_cols);

    Mat output;
    Mat big_matrix_combined1_photo = big_matrix_combined.clone();
    save_matrix_with_zeros(big_matrix_combined1_photo, "../data/deprojected_points0.txt", num_rows, num_cols, o_camera_position);
    normalizeAndInvert(big_matrix_combined1_photo, output);
    imwrite("../data/deprojected_image0.png", output);
    
    char deprojected_filename[100];
    if(n_images != 1){
        for(int n_image = 1; n_image < n_images; n_image++){
            Mat matrix_to_be_merged = Mat::zeros(num_rows, num_cols, CV_32SC1);
            Vector3f camera_position = camera_position = populate_matrix_from_file(filenames[n_image].c_str(), matrix_to_be_merged, center_y, center_x, cell_dim, num_rows, num_cols);
            Mat big_matrix_combined2_photo = matrix_to_be_merged.clone();
            sprintf(deprojected_filename, "../data/deprojected_points%d.txt", n_image);
            save_matrix_with_zeros(big_matrix_combined2_photo, deprojected_filename, num_rows, num_cols, camera_position);
            normalizeAndInvert(big_matrix_combined2_photo, output);
            sprintf(deprojected_filename, "../data/deprojected_image%d.png", n_image);
            imwrite(deprojected_filename, output);

            if(check_matrix(big_matrix_combined, matrix_to_be_merged, num_rows, num_cols, e)){
                populate_matrix_from_file(filenames[n_image].c_str(), big_matrix_combined, center_y, center_x, cell_dim, num_rows, num_cols);
                cout << "Image " << n_image << " merged" << endl;
            }
            else{
                cout << "Images too diferent to be merged" << endl;
            }
        }
    }
    else{
        cout << "Only one image" << endl;
    }

    save_matrix_with_zeros(big_matrix_combined, "../data/combinated_deprojected_points.txt", num_rows, num_cols, o_camera_position);
    normalizeAndInvert(big_matrix_combined, output);
    imwrite("../data/combinated_deprojected_image.png", output);
    
    system("source ~/Desktop/robotics_project/.venv/bin/activate");
    system("python ../hystogram.py");
    return 0;
}
