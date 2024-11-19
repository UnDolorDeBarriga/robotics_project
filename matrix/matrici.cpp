#include "spatial_transf.hpp"
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;


int main(){
    // Matrice di rotazione
    Eigen::Matrix4d Rx = RotationMatrix(1, 0);  // Rotazione attorno all'asse X
    Eigen::Matrix4d Ry = RotationMatrix(2, 0);  // Rotazione attorno all'asse Y
    Eigen::Matrix4d Rz = RotationMatrix(3, 0);  // Rotazione attorno all'asse Z
    // Matrice di traslazione
    Eigen::Matrix4d T= TranslationMatrix(0, 0, 1000);  // Traslazione (1, 2, 3)
    // Matrice di trasformazione spaziale
    Eigen::Matrix4d M = T * Rx * Ry *Rz ;
    read_txt("../points_image0.txt","../out.txt",M);
    
    return 0;
}