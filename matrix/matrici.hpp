#ifndef MATRICI_HPP
#define MATRICI_HPP

class Vector
{
    protected:
        double* data_;
        int size_;
    public:
        Vector& operator=(const Vector& other);
        Vector operator+(const Vector& other);
        Vector& operator+=(const Vector& a);
        Vector(int size);
        int size();
        double get(int index) ;
        void set(int index, double value);
};

class Matrix:public Vector{
    private:
        int rows;
        int cols;
        double aux_prodotto(int r,Matrix a , Matrix b, int c);
    public:
        Matrix(int rows,int cols);
        double get(int r,int c) ;
        void set(int r, int c,double value);
        bool read_txt(char filename[]);
        void print();
        bool print_txt(char filename[]);
        void ones();
        void random();
        Matrix operator*(Matrix& a);
        double operator%(Matrix& a);
        Matrix transpose();
        void identityMatrix();
        void RotationMatrix(char axis, double angle);
        void traslationMatrix(double tx,double ty,double tz);
        void spatialMatrix( Matrix& R,Vector& t);
};

#endif