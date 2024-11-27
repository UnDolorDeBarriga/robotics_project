#include "spatial_transf.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>  // Include OpenCV header



// int main(){
//     // Matrice di rotazione
//     Eigen::Matrix4d Rx = RotationMatrix(1, 0);  // Rotazione attorno all'asse X
//     Eigen::Matrix4d Ry = RotationMatrix(2, 0);  // Rotazione attorno all'asse Y
//     Eigen::Matrix4d Rz = RotationMatrix(3, 0);  // Rotazione attorno all'asse Z
//     // Matrice di traslazione
//     Eigen::Matrix4d T= TranslationMatrix(0, 0, 1000);  // Traslazione (1, 2, 3)
//     // Matrice di trasformazione spaziale
//     Eigen::Matrix4d M = T * Rx * Ry *Rz ;
//     printf("hehahe");
//     read_txt("../reference_points_image0.txt","../out.txt",M);
//     return 0;
// }

int main(){
    int center_x;
    int center_y;

    double maxAbsX=0;
    double maxAbsY=0;
    
    Eigen::Matrix4d Rx = RotationMatrix(1, 0);  // Rotazione attorno all'asse X
    Eigen::Matrix4d Ry = RotationMatrix(2, 0);  // Rotazione attorno all'asse Y
    Eigen::Matrix4d Rz = RotationMatrix(3, 0);  // Rotazione attorno all'asse Z
    // Matrice di traslazione
    Eigen::Matrix4d T= TranslationMatrix(0, 0, 1000);  // Traslazione (1, 2, 3)
    // Matrice di trasformazione spaziale
    Eigen::Matrix4d M = T * Rx * Ry *Rz ;
    read_txt("../camera_points_image0.txt","../reference_points_image0.txt",M, maxAbsX, maxAbsY);

    MatrixXd big_ass_matrix_combined = create_matrix(maxAbsX, maxAbsY, 1, center_y, center_x);

    
    MatrixXd big_ass_matrix1 = big_ass_matrix_combined;
    MatrixXd big_ass_matrix2 = big_ass_matrix_combined;

    printf("hehahe");
    populate_matrix_from_file("../reference_points_image0.txt", big_ass_matrix1, center_y, center_x, 1);
}