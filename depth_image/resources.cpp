#include "resources.h"
#include <fstream>
#include <iostream>
#include <sstream>



/**
 * @brief Captures depth frames and accumulates depth data.
 * 
 * This function captures a specified number of depth frames from a RealSense pipeline,
 * normalizes the depth values, and accumulates the depth data. It also counts the number
 * of valid depth measurements for each pixel.
 * 
 * @param pipeline The RealSense pipeline to capture frames from.
 * @param n_index The number of frames to capture.
 * @param accumulated_depth A matrix to accumulate the depth values.
 * @param valid_pixel_count A matrix to count the number of valid depth measurements for each pixel.
 * @param min_dist The minimum distance to consider for depth measurements (in meters).
 * @param max_dist The maximum distance to consider for depth measurements (in meters).
 * @return rs2_intrinsics The camera intrinsics of the captured frames.
 */
rs2_intrinsics get_main_frames_count(pipeline pipeline, int n_index, Mat &accumulated_depth, Mat &valid_pixel_count, int min_dist, int max_dist) {
    rs2_intrinsics intrinsics;
    int max_dist_mm = max_dist * 1000;
    for (int frame_count = 0; frame_count < n_index; ++frame_count) {
        // Wait for the next set of frames
        frameset frames = pipeline.wait_for_frames();
        depth_frame depth_frame = frames.get_depth_frame();
        intrinsics = depth_frame.get_profile().as<video_stream_profile>().get_intrinsics();

        //TODO: Make that saves the frames in an array, and does teh computation later, to make sure that the frames are teh continous
        Mat depth_data = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
        // Normalize depth values to max distance (in millimeters)
        for (int i = 0; i < WIDTH; i++){
            for (int j = 0; j < HEIGHT; j++){
                float depth = depth_frame.get_distance(i, j);
                if (depth >= min_dist) {
                    if (depth >= max_dist) {
                        depth_data.at<float>(j, i) = max_dist_mm;
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
    return intrinsics;
}

/**
 * @brief Writes depth data to files and performs transformation.
 *
 * This function processes depth data by computing the mean depth image, writing it to a CSV file,
 * deprojecting it into 3D points, and writing the depth image to a PNG file. It also creates a 
 * transformation matrix based on camera position and angle, and transforms coordinates accordingly.
 *
 * @param n_index Index of the current dataset.
 * @param image_n Index of the current image.
 * @param i_filename Input filename for depth data.
 * @param o_filename Output filename for transformed coordinates.
 * @param pos_filename Filename for camera position and angle data.
 * @param accumulated_depth Accumulated depth data.
 * @param valid_pixel_count Count of valid pixels in the depth data.
 * @param intrinsics Camera intrinsics for depth deprojection.
 * @param min_dist Minimum distance for depth values (in meters).
 * @param max_dist Maximum distance for depth values (in meters).
 * @param maxAbsX Maximum absolute X coordinate for transformation.
 * @param maxAbsY Maximum absolute Y coordinate for transformation.
 */
void write_data_to_files(int n_index, int image_n, const char i_filename[], const char o_filename[], const char pos_filename[], Mat accumulated_depth, Mat valid_pixel_count, rs2_intrinsics intrinsics, int min_dist, int max_dist, double maxAbsX, double maxAbsY) {

    // Compute the mean depth image
    Mat average_depth = get_mean_depth(accumulated_depth, valid_pixel_count, max_dist);
    printf("Computed mean depth for image %d\n", image_n);
    
    // Write the depth data to a CSV file
    write_depth_to_csv(average_depth, n_index, image_n);
    printf("Wrote depth data to CSV for image %d\n", image_n);
    
    // Deproject the mean depth image into 3D points
    auto points_image = deproject_depth_to_3d(i_filename, average_depth, intrinsics, image_n, min_dist, max_dist);
    printf("Deprojected depth to 3D for image %d\n", image_n);

    // Write the mean depth image to a PNG file
    write_depth_to_image(average_depth, (max_dist*1000), n_index, image_n);
    printf("Wrote depth image to PNG for image %d\n", image_n);
    
    // Get the user points for the camera position and angle
    Vector3f camera_position;
    Vector3f camera_angle;
    
    //get_user_points_input(image_n, camera_position, camera_angle);
    get_user_points_file(pos_filename, image_n, camera_position, camera_angle);
    cout << "Camera Position" << camera_position.transpose() << endl;
    cout << "Camera Angle" << camera_angle.transpose() << endl;



    Matrix4d M = create_transformation_matrix(camera_position, camera_angle);
    printf("Created transformation matrix for image %d\n", image_n);

    transformate_cordinates(i_filename, o_filename, M, maxAbsX, maxAbsY, camera_position, camera_angle);
    printf("Transformed coordinates for image %d\n", image_n);

    return;
}

/**
 * @brief Computes the mean depth for each pixel from accumulated depth values and valid pixel counts.
 *
 * This function calculates the average depth for each pixel by dividing the accumulated depth values
 * by the corresponding valid pixel counts. If a pixel has no valid counts, its average depth remains zero.
 *
 * @param accumulated_depth A matrix of accumulated depth values for each pixel (CV_32FC1).
 * @param valid_pixel_count A matrix of valid pixel counts for each pixel (CV_32SC1).
 * @return A matrix of average depth values for each pixel (CV_32FC1).
 */
Mat get_mean_depth(Mat accumulated_depth, Mat valid_pixel_count, int max_dist) {
    Mat average_depth = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
    float average_depth_point = 0.0f;
    for (int x = 0; x < accumulated_depth.cols; ++x) {
        for (int y = 0; y < accumulated_depth.rows; ++y) {
            if (valid_pixel_count.at<int>(y, x) > 0) {
                average_depth_point = accumulated_depth.at<float>(y, x) / valid_pixel_count.at<int>(y, x);
                if (average_depth_point >= 0.99*(max_dist*1000)){
                    average_depth.at<float>(y, x) = max_dist*1000; 
                } else {
                    average_depth.at<float>(y, x) = accumulated_depth.at<float>(y, x) / valid_pixel_count.at<int>(y, x);
                }
            }
        }
    }
    return average_depth;
}

/**
 * @brief Writes the depth matrix data to a CSV file.
 * 
 * @param depth_matrix The depth matrix data.
 * @param n_index The index of the current image.
 * @param image_n The image number.
 */
void write_depth_to_csv(Mat depth_matrix, int n_index, int image_n) {
    char filename[50];
    sprintf(filename, "../data/mean%d_depth%d.csv", n_index, image_n);
    ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        printf("Failed to open the CSV file.\n");
        return;
    }
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            float depth_mm = depth_matrix.at<float>(y, x);
            csv_file << depth_mm << ",";
        }
        csv_file << "\n";
    }
    csv_file.close();
    return;
}

/**
 * @brief Deprojects the depth matrix into 3D points.
 * 
 * @param i_filename The input file name to save the 3D coordinates.
 * @param depth_matrix The depth matrix data.
 * @param intrinsics The camera intrinsics.
 * @param image_n The image number.
 * @param min_dist The minimum distance for depth values (in meters).
 * @param max_dist The maximum distance for depth values (in meters).
 * @return std::vector<Eigen::Vector3f> The 3D points.
 */
vector<Vector3f> deproject_depth_to_3d(const char i_filename[], Mat depth_matrix, rs2_intrinsics intrinsics, int image_n, int min_dist, int max_dist) {
    vector<Eigen::Vector3f> points;
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            float depth = depth_matrix.at<float>(y, x);
            if (depth > (min_dist*1000) && depth < (max_dist*1000)) {
                float point[3];
                float pixel[2] = { static_cast<float>(x), static_cast<float>(y) };
                rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth);
                points.emplace_back(point[0], point[1], point[2]);
            }
        }
    }
    ofstream points_file(i_filename);
    for (const auto& point : points) {
        points_file << point[0] << "," << point[1] << "," << point[2] << "\n";
    }
    points_file.close();
    return points;
}



/**
 * @brief Writes the depth_matrix data to an image file.
 * 
 * @param depth_matrix The depth matrix data.
 * @param max_depth The maximum depth value.
 * @param n_index The index of the current depth image.
 * @param image_n The image number.
 */
void write_depth_to_image(Mat depth_matrix, int max_depth, int n_index, int image_n) {
    Mat mean_depth_image;
    depth_matrix.convertTo(mean_depth_image, CV_8UC1, 255.0 / (max_depth), -255.0 / (max_depth));
    Mat depth_color;
    applyColorMap(mean_depth_image, depth_color, COLORMAP_JET);
    char filename[50];
    sprintf(filename, "../data/mean%d_depth_image%d.png", n_index, image_n);
    imwrite(filename, depth_color);
    return;
}

/**
 * @brief Gets the user-defined points for the camera position and angle.
 * 
 * @param image_n The image number.
 * @param camera_position Pointer to the camera position.
 * @param camera_angle Pointer to the camera angle.
 */
void get_user_points_input(int image_n, Vector3f &camera_position, Vector3f &camera_angle) {
    vector<Eigen::Vector3f> user_points;
    for (int i = 0; i < 2; ++i) {
        while (true) {
            string input;
            if (i == 0) {
                cout << "Enter the Position of the camera in the format x,y,z: ";
            } else {
                cout << "Enter the Angle of the camera pitch,yaw,roll: ";
            }
            getline(cin, input);
            stringstream ss(input);
            string item;
            vector<float> point_values;
            while (getline(ss, item, ',')) {
                try {
                    if (i == 1 && stof(item) > 360) {
                        cout << "Angle must be between 0 and 360\n";
                        break;
                    }
                    point_values.push_back(stof(item));
                } catch (const invalid_argument&) {
                    break;
                }
            }
            if (point_values.size() == 3) {
                user_points.emplace_back(point_values[0], point_values[1], point_values[2]);
                break;
            }
            cout << "Invalid input. Please enter float values in the format x,y,z.\n";
            point_values.clear();
        }
    }
    camera_position = user_points[0];
    camera_angle = user_points[1];
    cout << "\nIMAGE " << image_n << " is from: ";
    cout << "\nCamera position: ";
    cout << camera_position.transpose() << "\n";
    cout << "Camera angle: ";
    cout << camera_angle.transpose() << "\n";
    return;
}




/**
 * @brief Reads the camera position and angle from a file and stores them in the provided vectors.
 *
 * This function opens a file specified by the given filename, reads the camera position and angle
 * from the first two lines of the file, and stores the values in the provided Vector3f objects.
 *
 * @param pos_filename The path to the file containing the camera position and angle.
 * @param image_n An integer representing the image number (not used in the function).
 * @param camera_position A reference to a Vector3f object where the camera position will be stored.
 * @param camera_angle A reference to a Vector3f object where the camera angle will be stored.
 */
void get_user_points_file(const char pos_filename[], int image_n, Vector3f &camera_position, Vector3f &camera_angle) {
    ifstream file(pos_filename);
    if (!file.is_open()) {
        cerr << "Error opening position file!" << endl;
        return;
    }
    string line;
    getline(file, line);
    stringstream ss_position(line);
    ss_position >> camera_position(0);
    ss_position.ignore(1); // Ignore the comma
    ss_position >> camera_position(1);
    ss_position.ignore(1); // Ignore the comma
    ss_position >> camera_position(2);

    getline(file, line);
    stringstream ss_angle(line);
    ss_angle >> camera_angle(0);
    ss_angle.ignore(1); // Ignore the comma
    ss_angle >> camera_angle(1);
    ss_angle.ignore(1); // Ignore the comma
    ss_angle >> camera_angle(2);
    file.close();
    return;
}



/**
 * @brief Generates a 4x4 rotation matrix for a given axis and angle.
 *
 * This function creates a 4x4 homogeneous rotation matrix that represents
 * a rotation around one of the principal axes (x, y, or z) by a specified angle.
 *
 * @param axis The axis of rotation (1 for x-axis, 2 for y-axis, 3 for z-axis).
 * @param angle The angle of rotation in degrees.
 * @return Eigen::Matrix4d The resulting 4x4 rotation matrix.
 * @throws std::invalid_argument if the axis is not 1, 2, or 3.
 */
Matrix4d rotation_matrix(int axis, double angle) {
    double rad = angle * M_PI / 180.0;
    Matrix4d rotation = Matrix4d::Identity();
    switch (axis) {
        case 1:
            rotation(1, 1) = cos(rad);
            rotation(1, 2) = -sin(rad);
            rotation(2, 1) = sin(rad);
            rotation(2, 2) = cos(rad);
            break;
        case 2:
            rotation(0, 0) = cos(rad);
            rotation(0, 2) = sin(rad);
            rotation(2, 0) = -sin(rad);
            rotation(2, 2) = cos(rad);
            break;
        case 3:
            rotation(0, 0) = cos(rad);
            rotation(0, 1) = -sin(rad);
            rotation(1, 0) = sin(rad);
            rotation(1, 1) = cos(rad);
            break; 
        default:
            throw invalid_argument("Axis must be '1', '2', or '3'");
    }
    return rotation;
}

/**
 * @brief Creates a 4x4 translation matrix.
 *
 * This function generates a 4x4 homogeneous transformation matrix that represents
 * a translation by the given distances along the X, Y, and Z axes.
 *
 * @param tx The translation distance along the X axis.
 * @param ty The translation distance along the Y axis.
 * @param tz The translation distance along the Z axis.
 * @return Eigen::Matrix4d A 4x4 matrix representing the translation.
 */
Matrix4d translation_matrix(double tx, double ty, double tz) {
    Matrix4d translation = Matrix4d::Identity();
    translation(0, 3) = tx;
    translation(1, 3) = ty;
    translation(2, 3) = tz;
    return translation;
}


/**
 * @brief Reads a text file containing 3D coordinates, applies a transformation matrix, and writes the transformed coordinates to an output file.
 * 
 * This function reads 3D coordinates from an input text file, applies a 4x4 transformation matrix to each coordinate, and writes the transformed coordinates to an output text file. It also calculates the maximum absolute values of the transformed x and y coordinates.
 * 
 * @param i_filename The path to the input text file containing 3D coordinates. Each line should contain three comma-separated values representing x, y, and z coordinates.
 * @param o_filename The path to the output text file where the transformed coordinates will be written. Each line will contain three comma-separated values representing the transformed x, y, and z coordinates.
 * @param M A 4x4 transformation matrix to be applied to each 3D coordinate.
 * @param maxAbsX A reference to a double variable where the maximum absolute value of the transformed x coordinates will be stored.
 * @param maxAbsY A reference to a double variable where the maximum absolute value of the transformed y coordinates will be stored.
 * @param camera_position The camera position vector.
 * @param camera_angle The camera angle vector.
 * 
 * @throws ios_base::failure If the input file cannot be opened.
 */
void transformate_cordinates(const char i_filename[],const char o_filename[], Matrix4d M, double& maxAbsX, double& maxAbsY,  Vector3f camera_position, Vector3f camera_angle) {
    ifstream myin;
    try {
        myin.open(i_filename);
        if (!myin) {
            throw ios_base::failure("Unable to open input file");
        }
    } catch (const ios_base::failure& e) {
        cerr << e.what() << endl;
        return;
    }
    ofstream myout;
    myout.open(o_filename);
    myout << camera_position(0) << "," << camera_position(1) << "," << camera_position(2) << endl;
    myout << camera_angle(0) << "," << camera_angle(1) << "," << camera_angle(2) << endl;
    string line;
    while (getline(myin, line)) {
        stringstream ss(line);  
        string temp;
        Vector4d vec;
        vec(3)=1;
        getline(ss, temp, ',');
        vec(0) = stod(temp); 
        getline(ss, temp, ',');
        vec(1) = stod(temp);
        getline(ss, temp, ',');
        vec(2) = stod(temp);
        vec=M*vec;
        myout << vec(0) << "," << vec(1) << "," << vec(2) << endl;
        if(std::abs(vec(0))>maxAbsX){
            maxAbsX = std::abs(vec(0));
        }
        if(std::abs(vec(1)) > maxAbsY){
            maxAbsY = std::abs(vec(1));
        }
    }
    myin.close();
    myout.close();
    return;
}



/**
 * @brief Creates a transformation matrix from camera position and angle.
 * 
 * This function generates a 4x4 transformation matrix that combines translation and rotation.
 * The rotation is applied in the order of x-axis, y-axis, and z-axis rotations, followed by translation.
 * 
 * @param camera_position A 3D vector representing the camera position (x, y, z).
 * @param camera_angle A 3D vector representing the camera rotation angles (pitch, yaw, roll) in degrees.
 * @return Eigen::Matrix4d The resulting 4x4 transformation matrix.
 */
Matrix4d create_transformation_matrix(Vector3f camera_position, Vector3f camera_angle){
    Matrix4d Rx = rotation_matrix(1, -90 - camera_angle[0]);
    Matrix4d Ry = rotation_matrix(2, -camera_angle[1]);
    Matrix4d Rz = rotation_matrix(3, -camera_angle[2]);

    Matrix4d T= translation_matrix(-camera_position[0], -camera_position[1], -camera_position[2]);
    Matrix4d M = T * Rz * Ry *Rx;
    return M;
}



/**
 * @brief Creates a matrix with specified dimensions and calculates the center point.
 *
 * This function generates a matrix of zeros with dimensions based on the provided
 * maximum absolute values for X and Y coordinates and the cell dimension. It also
 * calculates the center point of the matrix, which corresponds to the origin (x = 0, y = 0).
 *
 * @param maxAbsX The maximum absolute value for the X coordinate.
 * @param maxAbsY The maximum absolute value for the Y coordinate.
 * @param cell_dim The dimension of each cell in the matrix.
 * @param center_point_row Reference to an integer where the row index of the center point will be stored.
 * @param center_point_col Reference to an integer where the column index of the center point will be stored.
 * @return MatrixXd A matrix of zeros with the calculated dimensions.
 */
MatrixXd create_matrix(double maxAbsX, double maxAbsY, int cell_dim, int& center_point_row, int& center_point_col) {
    int num_rows = ceil((2 * maxAbsY) / cell_dim);
    int num_cols = ceil((2 * maxAbsX) / cell_dim);
    MatrixXd z_matrix = MatrixXd::Zero(num_rows+1, num_cols+1);

    // Determine the center point in the matrix (corresponding to x = 0, y = 0)
    center_point_row = num_rows / 2;
    center_point_col = num_cols / 2;
    return z_matrix;
}




/**
 * @brief Populates a matrix from a file containing 3D coordinates.
 * 
 * This function reads 3D coordinates from an input file and populates a matrix with the z-values.
 * The matrix is centered around a specified center point, and the coordinates are scaled based on the cell dimension.
 * 
 * @param i_filename The path to the input file containing 3D coordinates.
 * @param matrix The matrix to be populated with z-values.
 * @param center_point_row The row index of the center point in the matrix.
 * @param center_point_col The column index of the center point in the matrix.
 * @param cell_dim The dimension of each cell in the matrix.
 */
void populate_matrix_from_file(const char i_filename[], MatrixXd& matrix, int center_point_row, int center_point_col, int cell_dim) {
    ifstream file(i_filename);
    if (!file.is_open()) {
        cerr << "Error opening file!" << endl;
        return;
    }
    int row, col;
    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        double x, y, z;
        char comma1, comma2;
        if (!(iss >> x >> comma1 >> y >> comma2 >> z) || comma1 != ',' || comma2 != ',') {
            cerr << "Invalid line format: " << line << endl;
            continue;
        }

        col = center_point_col + static_cast<int>(floor(x / cell_dim));
        row = center_point_row - static_cast<int>(floor(y / cell_dim));
 

        if (row >= 0 && row < matrix.rows() && col >= 0 && col < matrix.cols()) {
            matrix(row, col) = static_cast<int>(round(z)); // Assign the z value to the appropriate cell
        } else {
            cerr << "Coordinates (" << x << ", " << y << ") out of matrix bounds. (row: "<< row <<" col: " <<col<<")" << endl;
        }
    }
    file.close();
}