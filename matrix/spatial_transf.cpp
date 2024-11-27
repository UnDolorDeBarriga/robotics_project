
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

void read_txt(const char i_filename[],const char o_filename[],Eigen::Matrix4d M, double& maxAbsX, double& maxAbsY){
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
        if(std::abs(vec(0))>maxAbsX){
            maxAbsX = std::abs(vec(0));
        }
        if(std::abs(vec(1)) > maxAbsY){
            maxAbsY = std::abs(vec(1));
        }
    }
    myin.close();
    myout.close();
    return;
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

void populate_matrix_from_file(const char i_filename[], MatrixXd& matrix, int center_point_row, int center_point_col, int cell_dim) {
    ifstream file(i_filename);
    if (!file.is_open()) {
        cerr << "Error opening file!" << endl;
        return;
    }
    int row, col;
    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        double x, y, z;
        char comma1, comma2;
        if (!(iss >> x >> comma1 >> y >> comma2 >> z) || comma1 != ',' || comma2 != ',') {
            cerr << "Invalid line format: " << line << endl;
            continue;
        }

        col = center_point_col + static_cast<int>(floor(x / cell_dim));
        row = center_point_row - static_cast<int>(floor(y / cell_dim));
 

        if (row >= 0 && row < matrix.rows() && col >= 0 && col < matrix.cols()) {
            matrix(row, col) = static_cast<int>(round(z)); // Assign the z value to the appropriate cell
        } else {
            cerr << "Coordinates (" << x << ", " << y << ") out of matrix bounds. (row: "<< row <<" col: " <<col<<")" << endl;
        }
    }
    file.close();
}