#include "matrici.hpp"
#include <iostream>
using namespace std;


// Constructor for the Vector class
// Initializes the size and allocates memory for the data array
Vector::Vector(int size): size_(size) {
    data_ = new double[size];
}
// Function to return the size of the Vector
int Vector::size() {
    return size_;
}

// Function to get the value at a specific index
double Vector::get (int index) {
    return data_[index];
}
// Function to set the value at a specific index
void Vector::set(int index, double value) {
    data_[index] = value;
}

// Overload the assignment operator
// Checks for self-assignment and allocates new memory if the sizes are different
Vector& Vector::operator=(const Vector& other) {
    if (this != &other) { // self-assignment check
        if (other.size_ != size_) {// storage cannot be reused
            delete[] data_; // destroy storage in this
            data_ = new double[other.size_]; // create storage in this
            size_ = other.size_;
        }
        std::copy(other.data_, other.data_ + other.size_, data_);
    }
    return *this;
}
// Overload the addition operator
// Creates a new Vector and adds the corresponding elements of the two Vectors
Vector Vector::operator+(const Vector& a) {
    Vector res(size_);
    for(int i=0; i<size_; ++i) {
        res.set(i, data_[i] + a.data_[i]);
    }
    return res;
}
// Overload the compound addition operator
// Adds the corresponding elements of the two Vectors and updates the current Vector
Vector& Vector::operator+=(const Vector& a) {
    for(int i=0; i<size_; ++i) {
        data_[i] += a.data_[i];
    }
    return *this;
}
