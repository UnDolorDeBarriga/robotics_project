#include "matrici.hpp"
#include <iostream>
using namespace std;

int main(int argc, char * argv[]){
    //rotation matrix
    Matrix  Rx1(4,4);
    Rx1.RotationMatrix('x', 0);
    Matrix  Ry1(4,4);
    Ry1.RotationMatrix('y', 0);   
    Matrix  Rz1(4,4);
    Rz1.RotationMatrix('z', 0);
    //translation matrix
    Matrix  T(4,4);
    T.traslationMatrix(1, 2, 3);
    //spatial transformation matrix
    Matrix M(4,4);
    M=T*Rx1*Ry1*Rz1;
    M.print();
    //point declaration
    Matrix P(4,1);
    P.set(0,0,2);
    P.set(0,1,2);
    P.set(0,2,2);
    //4d vector with last value 1
    P.set(0,3,1);
    P.print();
    //apply trasformation
    P= M * P;
    P.print();
}