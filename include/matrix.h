#pragma once

#include <stddef.h>

template <typename T, size_t Rows, size_t Cols>
class Matrix {
   public:
    static const size_t kRows = Rows;
    static const size_t kCols = Cols;
    static const size_t kSize = Rows * Cols > 0 ? Rows* Cols : 1;

    Matrix();
    explicit Matrix(T value);
    Matrix(const T* values, size_t count);
    Matrix(const Matrix& other);

    void setZero();
    void fill(T value);
    void assign(const T* values, size_t count);

    void setIdentity();
    static Matrix Identity();
    static Matrix Zero();

    T& operator()(size_t row, size_t col);
    const T& operator()(size_t row, size_t col) const;

    const T* raw() const;
    T* raw();

    Matrix operator+(const Matrix& rhs) const;
    Matrix& operator+=(const Matrix& rhs);
    Matrix operator-(const Matrix& rhs) const;
    Matrix& operator-=(const Matrix& rhs);
    Matrix operator*(T scalar) const;
    Matrix& operator*=(T scalar);
    Matrix operator/(T scalar) const;
    Matrix& operator/=(T scalar);

    template <size_t OtherCols>
    Matrix<T, Rows, OtherCols> operator*(
        const Matrix<T, Cols, OtherCols>& rhs) const;

    Matrix<T, Cols, Rows> transpose() const;

    bool inverse(Matrix& out, T epsilon = static_cast<T>(1e-6)) const;

    Matrix& operator=(const Matrix& rhs);

   private:
    static T absolute(T value);
    static void swapRows(Matrix& matrix, size_t row_a, size_t row_b);
    static void scaleRow(Matrix& matrix, size_t row, T scalar);
    static void subtractRow(Matrix& matrix, size_t target, size_t pivot,
                            T factor);
    static bool tryCholeskyInverse(const Matrix& input, Matrix& out,
                                   T epsilon);
    static bool isSymmetricMatrix(const Matrix& matrix, T epsilon);
    static bool choleskyDecompose(Matrix& matrix, T* diag, T epsilon);
    static void choleskyInvertLower(Matrix& matrix, const T* diag);
    static void choleskyCompleteInverse(Matrix& matrix);
    static T fastInverseSqrt(T value);
    static T fastSqrt(T value);

    T data_[kSize];
};

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> operator*(T scalar, const Matrix<T, Rows, Cols>& matrix);
