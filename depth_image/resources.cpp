#include "resources.h"
#include <fstream>

using namespace Eigen;
using namespace std;

/**
 * @brief Writes the depth matrix data to a CSV file.
 * 
 * @param depth_matrix The depth matrix data.
 * @param n_index The index of the current image.
 * @param image_n The image number.
 */
void write_depth_to_csv(cv::Mat depth_matrix, int n_index, int image_n) {
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
 * @return std::vector<Eigen::Vector3f> The 3D points.
 */
vector<Eigen::Vector3f> deproject_depth_to_3d(const char i_filename[], cv::Mat depth_matrix, rs2_intrinsics intrinsics, int image_n) {
    vector<Eigen::Vector3f> points;
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            float depth = depth_matrix.at<float>(y, x);
            if (depth > 0) {
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
 * @brief Computes the mean depth image.
 * 
 * @param accumulated_depth The depth matrix to be averages.
 * @param valid_pixel_count The matrix of number of valid pixels.
 */
void get_mean_depth(cv::Mat &accumulated_depth, cv::Mat valid_pixel_count) {
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            if (valid_pixel_count.at<int>(y, x) > 0) {
                accumulated_depth.at<float>(y, x) /= valid_pixel_count.at<int>(y, x);
            }
        }
    }
    return;
}

/**
 * @brief Writes the depth_matrix data to an image file.
 * 
 * @param depth_matrix The depth matrix data.
 * @param max_depth The maximum depth value.
 * @param n_index The index of the current depth image.
 * @param image_n The image number.
 */
void write_depth_to_image(cv::Mat depth_matrix, int max_depth, int n_index, int image_n) {
    cv::Mat mean_depth_image;
    depth_matrix.convertTo(mean_depth_image, CV_8UC1, 255.0 / (max_depth), -255.0 / (max_depth));
    cv::Mat depth_color;
    cv::applyColorMap(mean_depth_image, depth_color, cv::COLORMAP_JET);
    char filename[50];
    sprintf(filename, "../data/mean%d_depth_image%d.png", n_index, image_n);
    cv::imwrite(filename, depth_color);
    return;
}

/**
 * @brief Gets the user-defined points for the camera position and angle.
 * 
 * @param image_n The image number.
 * @param camera_position Pointer to the camera position.
 * @param camera_angle Pointer to the camera angle.
 */
void get_user_points(int image_n, Vector3f &camera_position, Vector3f &camera_angle) {
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
Matrix4d RotationMatrix(int axis, double angle) {
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
Matrix4d TranslationMatrix(double tx, double ty, double tz) {
    Matrix4d translation = Matrix4d::Identity();
    translation(0, 3) = tx;
    translation(1, 3) = ty;
    translation(2, 3) = tz;
    return translation;
}


/**
 * @brief Reads a text file containing 3D coordinates, applies a transformation matrix, and writes the transformed coordinates to an output file.
 * 
 * @param i_filename The input file name containing 3D camera cordinated system.
 * @param o_filename The output file name where the transformed coordinates in the set position cordinates.
 * @param M The 4x4 transformation matrix to be applied to the 3D coordinates.
 * 
 * The function reads each line from the input file, parses the 3D coordinates, applies the transformation matrix, 
 * and writes the transformed coordinates to the output file. If the input file cannot be opened, an error message is printed.
 */
void read_txt(const char i_filename[],const char o_filename[], Matrix4d M) {
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
    string line;
    while (getline(myin, line)) {
        stringstream ss(line);  
        string temp;
        Eigen::Vector4d vec;
        vec(3)=1;
        getline(ss, temp, ',');
        vec(0) = stod(temp); 
        getline(ss, temp, ',');
        vec(1) = stod(temp);
        getline(ss, temp, ',');
        vec(2) = stod(temp);
        vec=M*vec;
        myout << vec(0) << "," << vec(1) << "," << vec(2) << endl;
    }
    myin.close();
    myout.close();
    return;
}