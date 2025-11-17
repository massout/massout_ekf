#include "matrix.h"

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>::Matrix() {
    setZero();
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>::Matrix(T value) {
    fill(value);
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>::Matrix(const T* values, size_t count) {
    assign(values, count);
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>::Matrix(const Matrix& other) {
    assign(other.data_, Rows * Cols);
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::setZero() {
    for (size_t i = 0; i < Rows * Cols; ++i) {
        data_[i] = static_cast<T>(0);
    }
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::fill(T value) {
    for (size_t i = 0; i < Rows * Cols; ++i) {
        data_[i] = value;
    }
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::assign(const T* values, size_t count) {
    size_t limit = Rows * Cols;
    if (count < limit) {
        limit = count;
    }
    for (size_t i = 0; i < limit; ++i) {
        data_[i] = values[i];
    }
    for (size_t i = limit; i < Rows * Cols; ++i) {
        data_[i] = static_cast<T>(0);
    }
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::setIdentity() {
    setZero();
    const size_t n = Rows < Cols ? Rows : Cols;
    for (size_t i = 0; i < n; ++i) {
        (*this)(i, i) = static_cast<T>(1);
    }
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> Matrix<T, Rows, Cols>::Identity() {
    Matrix identity;
    identity.setIdentity();
    return identity;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> Matrix<T, Rows, Cols>::Zero() {
    Matrix zero;
    zero.setZero();
    return zero;
}

template <typename T, size_t Rows, size_t Cols>
T& Matrix<T, Rows, Cols>::operator()(size_t row, size_t col) {
    return data_[row * Cols + col];
}

template <typename T, size_t Rows, size_t Cols>
const T& Matrix<T, Rows, Cols>::operator()(size_t row, size_t col) const {
    return data_[row * Cols + col];
}

template <typename T, size_t Rows, size_t Cols>
const T* Matrix<T, Rows, Cols>::raw() const {
    return data_;
}

template <typename T, size_t Rows, size_t Cols>
T* Matrix<T, Rows, Cols>::raw() {
    return data_;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> Matrix<T, Rows, Cols>::operator+(
    const Matrix& rhs) const {
    Matrix result;
    for (size_t i = 0; i < Rows * Cols; ++i) {
        result.data_[i] = data_[i] + rhs.data_[i];
    }
    return result;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>& Matrix<T, Rows, Cols>::operator+=(const Matrix& rhs) {
    for (size_t i = 0; i < Rows * Cols; ++i) {
        data_[i] += rhs.data_[i];
    }
    return *this;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> Matrix<T, Rows, Cols>::operator-(
    const Matrix& rhs) const {
    Matrix result;
    for (size_t i = 0; i < Rows * Cols; ++i) {
        result.data_[i] = data_[i] - rhs.data_[i];
    }
    return result;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>& Matrix<T, Rows, Cols>::operator-=(const Matrix& rhs) {
    for (size_t i = 0; i < Rows * Cols; ++i) {
        data_[i] -= rhs.data_[i];
    }
    return *this;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> Matrix<T, Rows, Cols>::operator*(T scalar) const {
    Matrix result;
    for (size_t i = 0; i < Rows * Cols; ++i) {
        result.data_[i] = data_[i] * scalar;
    }
    return result;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>& Matrix<T, Rows, Cols>::operator*=(T scalar) {
    for (size_t i = 0; i < Rows * Cols; ++i) {
        data_[i] *= scalar;
    }
    return *this;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> Matrix<T, Rows, Cols>::operator/(T scalar) const {
    Matrix result;
    if (scalar == static_cast<T>(0)) {
        return result;
    }
    T inverse = static_cast<T>(1) / scalar;
    for (size_t i = 0; i < Rows * Cols; ++i) {
        result.data_[i] = data_[i] * inverse;
    }
    return result;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>& Matrix<T, Rows, Cols>::operator/=(T scalar) {
    if (scalar == static_cast<T>(0)) {
        return *this;
    }
    T inverse = static_cast<T>(1) / scalar;
    for (size_t i = 0; i < Rows * Cols; ++i) {
        data_[i] *= inverse;
    }
    return *this;
}

template <typename T, size_t Rows, size_t Cols>
template <size_t OtherCols>
Matrix<T, Rows, OtherCols> Matrix<T, Rows, Cols>::operator*(
    const Matrix<T, Cols, OtherCols>& rhs) const {
    Matrix<T, Rows, OtherCols> result;
    result.setZero();
    for (size_t r = 0; r < Rows; ++r) {
        for (size_t c = 0; c < OtherCols; ++c) {
            T sum = static_cast<T>(0);
            for (size_t k = 0; k < Cols; ++k) {
                sum += (*this)(r, k) * rhs(k, c);
            }
            result(r, c) = sum;
        }
    }
    return result;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Cols, Rows> Matrix<T, Rows, Cols>::transpose() const {
    Matrix<T, Cols, Rows> result;
    for (size_t r = 0; r < Rows; ++r) {
        for (size_t c = 0; c < Cols; ++c) {
            result(c, r) = (*this)(r, c);
        }
    }
    return result;
}

template <typename T, size_t Rows, size_t Cols>
bool Matrix<T, Rows, Cols>::inverse(Matrix& out, T epsilon) const {
    if (Rows != Cols) {
        return false;
    }

    Matrix working(*this);
    Matrix identity = Matrix::Identity();

    for (size_t i = 0; i < Rows; ++i) {
        size_t pivot = i;
        T pivot_value = absolute(working(i, i));
        for (size_t r = i + 1; r < Rows; ++r) {
            T candidate = absolute(working(r, i));
            if (candidate > pivot_value) {
                pivot_value = candidate;
                pivot = r;
            }
        }

        if (pivot_value <= epsilon) {
            return false;
        }

        if (pivot != i) {
            swapRows(working, i, pivot);
            swapRows(identity, i, pivot);
        }

        T diag = working(i, i);
        if (diag == static_cast<T>(0)) {
            return false;
        }
        T inv_diag = static_cast<T>(1) / diag;
        scaleRow(working, i, inv_diag);
        scaleRow(identity, i, inv_diag);

        for (size_t r = 0; r < Rows; ++r) {
            if (r == i) {
                continue;
            }
            T factor = working(r, i);
            if (factor != static_cast<T>(0)) {
                subtractRow(working, r, i, factor);
                subtractRow(identity, r, i, factor);
            }
        }
    }

    out = identity;
    return true;
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols>& Matrix<T, Rows, Cols>::operator=(const Matrix& rhs) {
    if (this != &rhs) {
        for (size_t i = 0; i < Rows * Cols; ++i) {
            data_[i] = rhs.data_[i];
        }
    }
    return *this;
}

template <typename T, size_t Rows, size_t Cols>
T Matrix<T, Rows, Cols>::absolute(T value) {
    return value < static_cast<T>(0) ? -value : value;
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::swapRows(Matrix& matrix, size_t row_a,
                                     size_t row_b) {
    for (size_t c = 0; c < Cols; ++c) {
        T temp = matrix(row_a, c);
        matrix(row_a, c) = matrix(row_b, c);
        matrix(row_b, c) = temp;
    }
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::scaleRow(Matrix& matrix, size_t row, T scalar) {
    for (size_t c = 0; c < Cols; ++c) {
        matrix(row, c) *= scalar;
    }
}

template <typename T, size_t Rows, size_t Cols>
void Matrix<T, Rows, Cols>::subtractRow(Matrix& matrix, size_t target,
                                        size_t pivot, T factor) {
    for (size_t c = 0; c < Cols; ++c) {
        matrix(target, c) -= factor * matrix(pivot, c);
    }
}

template <typename T, size_t Rows, size_t Cols>
Matrix<T, Rows, Cols> operator*(T scalar, const Matrix<T, Rows, Cols>& matrix) {
    return matrix * scalar;
}