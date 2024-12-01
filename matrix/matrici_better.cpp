#include "spatial_transf.hpp"
#include <iostream>
#include <Eigen/Dense>




int main(){
    int center_x;
    int center_y;

    int dim = 1;
    int n_lines1, n_lines2;
    double maxAbsX=0;
    double maxAbsY=0;
    
    Eigen::Matrix4d Rx = RotationMatrix(1, -90);  // Rotazione attorno all'asse X
    Eigen::Matrix4d Ry = RotationMatrix(2, 0);  // Rotazione attorno all'asse Y
    Eigen::Matrix4d Rz = RotationMatrix(3, 0);  // Rotazione attorno all'asse Z
    // Matrice di traslazione
    Eigen::Matrix4d T= TranslationMatrix(0, 0, 110);  // Traslazione (1, 2, 3)
    // Matrice di trasformazione spaziale
    Eigen::Matrix4d M = T * Rz * Ry *Rx ;

    n_lines1 = read_txt("../data/camera_points_image0.txt","../data/reference_points_image0.txt",M, maxAbsX, maxAbsY);
    //n_lines2 = read_txt("../data/camera_points_image1.txt","../data/reference_points_image1.txt",M, maxAbsX, maxAbsY);
    
    int num_rows = std::ceil((maxAbsY) / dim);
    int num_cols = std::ceil((2 * maxAbsX) / dim);
    
    cv::Mat big_matrix = cv::Mat::zeros(num_rows+1, num_cols+1, CV_8UC1);
    center_y = num_rows;
    center_x = num_cols / 2;


    int e = 20;
    int n_rows = big_matrix.rows;
    int n_cols = big_matrix.cols;

    populate_matrix_from_file_better("../data/reference_points_image0.txt", big_matrix, center_y, center_x, dim, n_rows, n_cols);

    int max = saveSparseMatrixWithZerosToTxt_better(big_matrix, "../data/big_matrix1_better.txt", n_rows, n_cols);

    for (int i = 0; i < n_rows; ++i) {
        for (int j = 0; j < n_cols; ++j) {
            if(big_matrix.at<uchar>(i, j) == 0) {
                big_matrix.at<uchar>(i, j) = 255;
            }
        }
    }
   
    cv::imwrite("../data/big_matrix_image.png", big_matrix);

    /*
        if (check_matrix(big_matrix1, big_matrix2, e)) {
            printf("4");
            merge_matrix(big_ass_matrix_combined, big_matrix1, big_matrix2);

            std::ofstream output_file("big_ass_matrix_combined.csv");
            for (int i = 0; i < big_ass_matrix_combined.rows(); ++i) {
                for (int j = 0; j < big_ass_matrix_combined.cols(); ++j) {
                    output_file << big_ass_matrix_combined.coeff(i, j);
                    if (j < big_ass_matrix_combined.cols() - 1) {
                        output_file << ",";
                    }
                }
                output_file << "\n";
            }
            output_file.close();
        }
        else {
            printf("5");
            printf("JAJA PRimo\n");
        }
    */

    #if False
        if (check_merge_matrix_with_file("../data/reference_points_image1.txt", big_ass_matrix_combined, center_y, center_x, dim, e)) {
        printf("4");
        //merge_matrix_with_file("../data/reference_points_image1.txt", big_ass_matrix_combined, center_y, center_x, 1);   
    }
    else {
        printf("5");
        printf("JAJA PRimo\n");
    }
#endif
    return 0;
}