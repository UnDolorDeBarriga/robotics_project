#ifndef SPATIAL_TRANSF_HPP
#define SPATIAL_TRANSF_HPP
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

Eigen::Matrix4d RotationMatrix(char axis, double angle);
Eigen::Matrix4d TranslationMatrix(double tx, double ty, double tz);
void read_txt(const char i_filename[],const char o_filename[],Eigen::Matrix4d M);
MatrixXd create_matrix(double maxAbsX, double maxAbsY, int cell_dim, int& center_point_row, int& center_point_col);

#endif