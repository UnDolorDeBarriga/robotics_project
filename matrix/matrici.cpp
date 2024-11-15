#include "matrici.hpp"
using namespace std;
#include <iostream>
#include <fstream>
#include <cmath>

Matrix::Matrix(int rows, int cols): Vector((rows)*(cols)) {      
    this ->  rows = rows;
    this ->  cols = cols;
}

double Matrix::get(int r,int c) {
    return Vector::get(r*(cols)+c);
};
void Matrix::set(int r, int c,double value){
    Vector::set(r*(cols)+c,value);
};

void Matrix::print(){
    int a=0;
    for(int i=0;i<size();i++){
        a++;
        cout<<Vector::get(i)<< " ";
        if(a%(cols)==0){
            cout<<"\n";
            a=0;
        }
    }
    cout<< endl;
};

bool Matrix::read_txt(char filename[]){
    fstream myin;
    myin.open(filename,ios::in);
    if (myin.fail()) {
        return false;
    }
    double value;
    int i=0;
    while (myin  >> value) {
        this->Vector::set(i, value);
        i++;
    }
    myin.close();
    return true;
}

bool Matrix::print_txt(char filename[]){
    fstream myapp;
    myapp.open(filename,ios::out|ios::app);
    if (myapp.fail()) {
        return false;
    }
    int a=0;
    for(int i=0;i<size();i++){
        a++;
        myapp<<Vector::get(i)<<" ";
        if(a%(cols)==0){
            myapp<<"\n";
            a=0;
        }
    }
    myapp.close();
    return true;
}

void Matrix::ones(){
    for(int i=0;i<size();i++){
        Vector::set(i,1);
    }
}

void Matrix::identityMatrix(){
    for(int i=0;i<rows;i++){
        for(int j=0;j < cols;j++)
        {
            if(i==j){
                Matrix::set(i,j,1);
            }
            else{
                Matrix::set(i,j,0);
            }
        
        }
    }
}

void Matrix::RotationMatrix(char axis, double angle){
    double rad = angle * M_PI / 180.0; // Convert angle to radians
    Matrix::identityMatrix();
    switch (axis) {
        case 'x': // Rotation around x-axis (pitch)
            Matrix::set(1,1,cos(rad));
            Matrix::set(1,2,sin(rad));
            Matrix::set(2,1,-sin(rad));
            Matrix::set(2,2,cos(rad));
            break;
        case 'y': // Rotation around y-axis (roll)
            Matrix::set(0,0,cos(rad));
            Matrix::set(0,2,-sin(rad));
            Matrix::set(2,0,sin(rad));
            Matrix::set(2,2,cos(rad));
            break;
        case 'z': // Rotation around z-axis (yaw)
            Matrix::set(0,0,cos(rad));
            Matrix::set(0,1,-sin(rad));
            Matrix::set(1,0,sin(rad));
            Matrix::set(1,1,cos(rad));
            break;
    }

}

void Matrix::traslationMatrix(double tx,double ty,double tz){
    Matrix::identityMatrix();
    Matrix::set(0,3,tx);
    Matrix::set(1,3,ty);
    Matrix::set(2,3,tz);

}

void Matrix::spatialMatrix( Matrix& R,Vector& t){
    Matrix::identityMatrix();
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            Matrix::set(i,j,R.get(i,j));
        }
    }
    for(int i=0;i<3;i++){
        Matrix::set(i,3,t.get(i));
    }
}
void Matrix::random(){
    for(int i=0;i<size();i++){
        Vector::set(i,rand());
    }
}

Matrix Matrix::operator*( Matrix& b){
    Matrix res(rows,b.cols);
    if (cols==b.rows){               
        for(int r=0;r<res.rows;r++){                                  //scorro array risultato per inserire valori
            for(int c=0;c<res.cols;c++){
                res.set(r,c,aux_prodotto(r,*this,b,c));
            }
        }
    }
    return res;

};


double Matrix::aux_prodotto(int r,Matrix a , Matrix b,int c){   
    double res=0;                                            //scorro array risultato per inserire valori
        for(int col=0;col<a.cols;col++){
            res=res+a.get(r,col)*b.get(col,c);  
        }
    return res;
};

double Matrix::operator%( Matrix& b){
   double res=0;
   for(int i=0;i<size();i++){
    res+=Vector::get(i)*b.Vector::get(i);
   }
   return res;

};


Matrix Matrix::transpose(){
    Matrix transpose(cols-1,rows-1);
    for(int r=0;r<transpose.rows;r++){                                  //scorro array risultato per inserire valori
        for(int c=0;c<transpose.cols;c++){

            transpose.set(r,c,get(c,r));
        }
    }
    return transpose;
};