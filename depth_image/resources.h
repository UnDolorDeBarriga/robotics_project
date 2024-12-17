#ifndef RESOURCES_H
#define RESOURCES_H

#include <iostream>
#include <vector>
#include <string>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <fstream>

// #define WIDTH 640
// #define HEIGHT 480

#define WIDTH 848
#define HEIGHT 480

#define FPS 10

#define MAX_ERROR 5

#define DEBUG 1

using namespace Eigen;
using namespace std;
using namespace rs2;
using namespace cv;

// Function declarations
rs2_intrinsics get_main_frames_count(pipeline pipeline, int n_index, Mat &accumulated_depth, Mat &valid_pixel_count, int min_dist, int max_dist);
void write_data_to_files(int n_index, int image_n, const char i_filename[], const char o_filename[], const char pos_filename[],
                         Mat accumulated_depth, Mat valid_pixel_count, rs2_intrinsics intrinsics, int min_dist, int max_dist, 
                         double& maxAbsX, double& maxAbsY);

void write_depth_to_csv(const Mat &depth_matrix, int n_index, int image_n);
vector<Vector3f> deproject_depth_to_3d(const char i_filename[], const Mat &depth_matrix, rs2_intrinsics intrinsics, int image_n, int min_dist, int max_dist);
Mat get_mean_depth(Mat accumulated_depth, Mat valid_pixel_count, int max_dist);
void write_depth_to_image(const Mat &depth_matrix, int max_depth, int n_index, int image_n);
void get_user_points_input(int image_n, Vector3f &camera_position, Vector3f &camera_angle);
void get_user_points_file(const char pos_filename[], int image_n, Vector3f &camera_position, Vector3f &camera_angle);

Matrix4d rotation_matrix(int axis, double angle);
Matrix4d translation_matrix(double tx, double ty, double tz);
void transformate_cordinates(const char i_filename[],const char o_filename[], Matrix4d M, double& maxAbsX, double& maxAbsY, Vector3f camera_position, Vector3f camera_angle);
Matrix4d create_transformation_matrix(Vector3f camera_position, Vector3f camera_angle);


Vector3f populate_matrix_from_file(const char i_filename[], cv::Mat& matrix, int center_point_row, int center_point_col, int cell_dim, int n_rows, int n_cols);
bool check_matrix(const Mat& matrix1, const Mat& matrix2, int n_rows, int n_cols, int e);
void save_matrix_with_zeros(const Mat& mat, const std::string& filename, int n_rows, int n_cols, Vector3f camera_position);
void normalizeAndInvert(const Mat& input, Mat& output);
#endif // RESOURCES_H



