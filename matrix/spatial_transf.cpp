#include "spatial_transf.hpp"




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

int read_txt(const char i_filename[], const char o_filename[], Eigen::Matrix4d M, double& maxAbsX, double& maxAbsY){
    ifstream myin;
    try {
        myin.open(i_filename);
        if (!myin) {
            throw std::ios_base::failure("Unable to open input file");
        }
    } catch (const std::ios_base::failure& e) {
        cerr << e.what() << endl;
        return -1;
    }
    ofstream myout;
    myout.open(o_filename);
    string line;
    int n_lines = 0;
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
        n_lines++;
    }
    myin.close();
    myout.close();
    return n_lines;
}

void populate_matrix_from_file(const char i_filename[], SparseMatrix<int>& matrix, int center_point_row, int center_point_col, int cell_dim, int n_lines) {
    ifstream file(i_filename);
    if (!file.is_open()) {
        cerr << "Error opening file!" << endl;
        return;
    }

    // Reserve space for non-zero elements
    matrix.reserve(n_lines);

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
            int z_value = static_cast<int>(round(z));
            if (matrix.coeff(row, col) == 0) {
                matrix.insert(row, col) = z_value;
            } else if (matrix.coeff(row, col) < z_value) {
                matrix.coeffRef(row, col) = z_value;
            }
        } else {
            cerr << "Coordinates (" << x << ", " << y << ") out of matrix bounds. (row: " << row << " col: " << col << ")" << endl;
        }
    }
    file.close();
    matrix.makeCompressed();
    return;
}

void merge_matrix_with_file(const char i_filename[], MatrixXd& matrix, int center_point_row, int center_point_col, int cell_dim) {
    ifstream file(i_filename);
    if (!file.is_open()) {
        cerr << "Error opening file!" << endl;
        return;
    }
    int row, col;
    int n=0;
    int squared_sum=0;
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
        

        if (row >= 0 && row < matrix.rows() && col >= 0 && col < matrix.cols() && matrix(row, col) == 0 && z != 0) {
            matrix(row, col) = static_cast<int>(round(z)); // Assign the z value to the appropriate cell
        } else if(row >= 0 && row < matrix.rows() && col >= 0 && col < matrix.cols() && matrix(row, col) != 0 && static_cast<int>(round(z)) > matrix(row, col)) {
            matrix(row, col) = static_cast<int>(round(z));
        } else if(row > matrix.rows()  || col > matrix.cols()){
            cerr << "Coordinates (" << x << ", " << y << ") out of matrix bounds. (row: "<< row <<" col: " <<col<<")" << endl;
        }
    }
    file.close();
}


bool check_merge_matrix_with_file(const char i_filename[], SparseMatrix<int>& matrix, int center_point_row, int center_point_col, int cell_dim, int e) {
    ifstream file(i_filename);
    if (!file.is_open()) {
        cerr << "Error opening file!" << endl;
        return false;
    }
    int row, col;
    int n=0;
    int squared_sum=0;
    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        double x, y, z;
        int temp_value;
        char comma1, comma2;
        if (!(iss >> x >> comma1 >> y >> comma2 >> z) || comma1 != ',' || comma2 != ',') {
            cerr << "Invalid line format: " << line << endl;
            continue;
        }
        col = center_point_col + static_cast<int>(floor(x / cell_dim));
        row = center_point_row - static_cast<int>(floor(y / cell_dim));
 
        if((row <= matrix.rows() || col <= matrix.cols()) && (row >= 0 && col >= 0)){
            temp_value = matrix.coeff(row, col);
            if(temp_value != 0 &&  z != 0) {
                n+=1;
                squared_sum += std::abs(std::pow(temp_value,2) - std::pow(static_cast<int>(round(z)),2));
                cout << "temp_value: " << temp_value << " z: " << z << " super z: " << static_cast<int>(round(z)) << endl;
            }   
        }
    }
    file.close();
    printf("n: %d\n",n);
    printf("squared_sum: %d\n",squared_sum);
    if(squared_sum/n < e){
        return true;
    } else {
        return false;
    }
}


bool check_matrix(SparseMatrix<int>& matrix1, SparseMatrix<int>& matrix2, int e) {
    int n=0;
    int squared_sum=0;
    for (int i = 0; i < matrix1.rows(); i++) {
        for (int j = 0; j < matrix1.cols(); j++) {
            int temp_value1 = matrix1.coeff(i, j);
            int temp_value2 = matrix2.coeff(i, j);
            if(temp_value1 != 0 && temp_value2 != 0) {
                n+=1;
                squared_sum += std::abs(std::pow(temp_value1,2) - std::pow(temp_value2,2));
            }
        }
    }
    if(squared_sum/n < e){
        return true;
    } else {
        return false;
    }
}


void merge_matrix(SparseMatrix<int>& big_matrix, SparseMatrix<int>& matrix1, SparseMatrix<int>& matrix2) {
    for (int i = 0; i < big_matrix.rows(); i++) {
        for (int j = 0; j < big_matrix.cols(); j++) {
            int temp_value1 = matrix1.coeff(i, j);
            int temp_value2 = matrix2.coeff(i, j);
            if(temp_value1 != 0 || temp_value2 != 0) {
                if (temp_value1 > temp_value2) {
                    big_matrix.insert(i, j) = temp_value1;
                } else {
                    big_matrix.insert(i, j) = temp_value2;
                }
            }
        }
    }
}