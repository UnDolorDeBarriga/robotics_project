#ifndef RESOURCES_H
#define RESOURCES_H

#include <iostream>
#include <vector>
#include <string>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <fstream>

#define MIN_DIST 0.01f 
#define WIDTH 640
#define HEIGHT 480
#define FPS 30

using namespace Eigen;
using namespace std;
using namespace rs2;
using namespace cv;

// Function declarations
void write_depth_to_csv(Mat depth_matrix, int n_index, int image_n);
vector<Vector3f> deproject_depth_to_3d(const char i_filename[], Mat depth_matrix, rs2_intrinsics intrinsics, int image_n);
Mat get_mean_depth(Mat accumulated_depth, Mat valid_pixel_count, float max_dist);
void write_depth_to_image(Mat depth_matrix, int max_depth, int n_index, int image_n);
void get_user_points(int image_n, Vector3f &camera_position, Vector3f &camera_angle);

Matrix4d rotation_matrix(int axis, double angle);
Matrix4d translation_matrix(double tx, double ty, double tz);
void transformate_cordinates(const char i_filename[],const char o_filename[], Matrix4d M, double& maxAbsX, double& maxAbsY, Vector3f camera_position, Vector3f camera_angle);
Matrix4d create_transformation_matrix(Vector3f camera_position, Vector3f camera_angle);

MatrixXd create_matrix(double maxAbsX, double maxAbsY, int cell_dim, int& center_point_row, int& center_point_col);
void populate_matrix_from_file(const char i_filename[], MatrixXd& matrix, int center_point_row, int center_point_col, int cell_dim);
#endif // RESOURCES_H



