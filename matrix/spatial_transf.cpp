
#include "spatial_transf.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <fstream>


Eigen::Matrix4d RotationMatrix(char axis, double angle) {
    double rad = angle * M_PI / 180.0; // Converti l'angolo in radianti
    Matrix4d rotation = Matrix4d::Identity(); // Matrice identità
    
    switch (axis) {
        case 1: // Rotazione attorno all'asse x (pitch)
            rotation(1, 1) = std::cos(rad);
            rotation(1, 2) = -std::sin(rad);
            rotation(2, 1) = std::sin(rad);
            rotation(2, 2) = std::cos(rad);
            break;
            
        case 2: // Rotazione attorno all'asse y (roll)
            rotation(0, 0) = std::cos(rad);
            rotation(0, 2) = std::sin(rad);
            rotation(2, 0) = -std::sin(rad);
            rotation(2, 2) = std::cos(rad);
            break;
            
        case 3: // Rotazione attorno all'asse z (yaw)
            rotation(0, 0) = std::cos(rad);
            rotation(0, 1) = -std::sin(rad);
            rotation(1, 0) = std::sin(rad);
            rotation(1, 1) = std::cos(rad);
            break;
            
        default:
            throw std::invalid_argument("Axis must be 'x', 'y', or 'z'");
    }
    
    return rotation;
}

Eigen::Matrix4d TranslationMatrix(double tx, double ty, double tz) {
    Matrix4d translation = Matrix4d::Identity();  // Inizializza la matrice identità 4x4  
    translation(0, 3) = tx;  // Imposta la traslazione lungo l'asse X
    translation(1, 3) = ty;  // Imposta la traslazione lungo l'asse Y
    translation(2, 3) = tz;  // Imposta la traslazione lungo l'asse Z  
    return translation;  // Restituisce la matrice di traslazione
}

void read_txt(const char i_filename[],const char o_filename[],Eigen::Matrix4d M){
    ifstream myin;
    try {
        myin.open(i_filename);
        if (!myin) {
            throw std::ios_base::failure("Unable to open input file");
        }
    } catch (const std::ios_base::failure& e) {
        cerr << e.what() << endl;
        return;
    }
    ofstream myout;
    myout.open(o_filename);
    string line;
    while (getline(myin, line)) {
        std::stringstream ss(line);  // Utilizziamo un stringstream per separare i valori
        std::string temp;
        // Crea un vettore Eigen a 3 dimensioni
        Eigen::Vector4d vec;
        vec(3)=1;
        // Leggi i tre valori separati da virgola
        getline(ss, temp, ',');
        vec(0) = std::stod(temp);  // Assegna il primo valore al vettore (x)
        getline(ss, temp, ',');
        vec(1) = std::stod(temp);  // Assegna il secondo valore al vettore (y)
        getline(ss, temp, ',');
        vec(2) = std::stod(temp);  // Assegna il terzo valore al vettore (z)
        vec=M*vec;
        myout << vec(0) << "," << vec(1) << "," << vec(2) << endl;
    }
    myin.close();
    myout.close();
}



MatrixXd create_matrix(double maxAbsX, double maxAbsY, int cell_dim, int& center_point_row, int& center_point_col) {
    int num_rows = ceil((2 * maxAbsY) / cell_dim);
    int num_cols = ceil((2 * maxAbsX) / cell_dim);
    MatrixXd z_matrix = MatrixXd::Zero(num_rows+1, num_cols+1);

    // Determine the center point in the matrix (corresponding to x = 0, y = 0)
    center_point_row = num_rows / 2;
    center_point_col = num_cols / 2;
    return z_matrix;
}