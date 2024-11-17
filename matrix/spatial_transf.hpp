#ifndef SPATIAL_TRANSF_HPP
#define SPATIAL_TRANSF_HPP
#include <Eigen/Dense>
Eigen::Matrix4d RotationMatrix(char axis, double angle);
Eigen::Matrix4d TranslationMatrix(double tx, double ty, double tz);
void read_txt(const char i_filename[],const char o_filename[],Eigen::Matrix4d M);

#endif