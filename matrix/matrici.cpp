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

    int dim = 1;
    int n_lines1, n_lines2;
    double maxAbsX=0;
    double maxAbsY=0;
    
    Eigen::Matrix4d Rx = RotationMatrix(1, 0);  // Rotazione attorno all'asse X
    Eigen::Matrix4d Ry = RotationMatrix(2, 0);  // Rotazione attorno all'asse Y
    Eigen::Matrix4d Rz = RotationMatrix(3, 0);  // Rotazione attorno all'asse Z
    // Matrice di traslazione
    Eigen::Matrix4d T= TranslationMatrix(0, 0, 1000);  // Traslazione (1, 2, 3)
    // Matrice di trasformazione spaziale
    Eigen::Matrix4d M = T * Rx * Ry *Rz ;

    n_lines1 = read_txt("../data/camera_points_image0.txt","../data/reference_points_image0.txt",M, maxAbsX, maxAbsY);
    n_lines2 = read_txt("../data/camera_points_image1.txt","../data/reference_points_image1.txt",M, maxAbsX, maxAbsY);
    
    int num_rows = std::ceil((2 * maxAbsY) / dim);
    int num_cols = std::ceil((2 * maxAbsX) / dim);
    
    SparseMatrix<int> big_ass_matrix_combined(num_rows+1, num_cols+1);
    SparseMatrix<int> big_matrix1(num_rows+1, num_cols+1);
    SparseMatrix<int> big_matrix2(num_rows+1, num_cols+1);
    center_y = num_rows / 2;
    center_x = num_cols / 2;


    int e = 20;
    populate_matrix_from_file("../data/reference_points_image0.txt", big_matrix1, center_y, center_x, dim, n_lines1);
    populate_matrix_from_file("../data/reference_points_image1.txt", big_matrix2, center_y, center_x, dim, n_lines2);
    if (check_merge_matrix_with_file("../data/reference_points_image1.txt", big_ass_matrix_combined, center_y, center_x, dim, e)) {
        printf("4");
        //merge_matrix_with_file("../data/reference_points_image1.txt", big_ass_matrix_combined, center_y, center_x, 1);   
    }
    else {
        printf("5");
        printf("JAJA PRimo\n");
    }

    return 0;
}