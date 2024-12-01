#ifndef SPATIAL_TRANSF_HPP
#define SPATIAL_TRANSF_HPP
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <opencv2/opencv.hpp>  // Include OpenCV header

using namespace Eigen;
using namespace std;

Eigen::Matrix4d RotationMatrix(char axis, double angle);
Eigen::Matrix4d TranslationMatrix(double tx, double ty, double tz);
int read_txt(const char i_filename[],const char o_filename[],Eigen::Matrix4d M, double& maxAbsX, double& maxAbsY);


void populate_matrix_from_file(const char i_filename[], SparseMatrix<int>& matrix, int center_point_row, int center_point_col, int cell_dim, int n_lines);
bool check_merge_matrix_with_file(const char i_filename[], SparseMatrix<int>& matrix, int center_point_row, int center_point_col, int cell_dim, int e);
void merge_matrix_with_file(const char i_filename[], MatrixXd& matrix, int center_point_row, int center_point_col, int cell_dim);   

void populate_matrix_from_file_better(const char i_filename[], cv::Mat& matrix, int center_point_row, int center_point_col, int cell_dim, int n_rows, int n_cols);
int saveSparseMatrixWithZerosToTxt_better(cv::Mat& mat, const std::string& filename, int n_rows, int n_cols);


bool check_matrix(SparseMatrix<int>& matrix1, SparseMatrix<int>& matrix2, int e);
void merge_matrix(SparseMatrix<int>& big_matrix, SparseMatrix<int>& matrix1, SparseMatrix<int>& matrix2);
void saveSparseMatrixWithZerosToTxt(Eigen::SparseMatrix<int>& mat, const std::string& filename);

#endif