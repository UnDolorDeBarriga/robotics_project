#ifndef RESOURCES_H
#define RESOURCES_H

#include <iostream>
#include <vector>
#include <string>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <fstream>

#define MIN_DIST 0.01f  // Set the max distance (in meters) for the color gradient
#define WIDTH 640
#define HEIGHT 480
#define FPS 30
#define N_IMAGES 2

using namespace Eigen;
using namespace std;

// Function declarations
void write_depth_to_csv(cv::Mat depth_matrix, int n_index, int image_n);
std::vector<Eigen::Vector3f> deproject_depth_to_3d(const char i_filename[], cv::Mat depth_matrix, rs2_intrinsics intrinsics, int image_n);
void get_mean_depth(cv::Mat &accumulated_depth, cv::Mat valid_pixel_count);
void write_depth_to_image(cv::Mat depth_matrix, int max_depth, int n_index, int image_n);
void get_user_points(int image_n, Vector3f &camera_position, Vector3f &camera_angle);

Matrix4d RotationMatrix(int axis, double angle);
Matrix4d TranslationMatrix(double tx, double ty, double tz);
void read_txt(const char i_filename[], const char o_filename[] Matrix4d M);

#endif // RESOURCES_H



