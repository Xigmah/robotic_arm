/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-17 19:35:10
 * @modify date 2026-02-17 19:35:10
 * @desc Code for Matrix4x4
 *       Used for conducting 4x4 math
 *       for calculating arm joint rotations/translations
 */

#include "matrix4x4.h"

#include <iostream>

/*
 * 4x4 transformation matrix encoding both rot and trans
 * (position shift):
 *
 *     ┌                    ┐
 *     │ R11  R12  R13  Tx  │
 * T = │ R21  R22  R23  Ty  │
 *     │ R31  R32  R33  Tz  │
 *     │  0    0    0    1  │
 *     └                    ┘
 */

namespace arm {
namespace math {

/* Default constructor - creates identity matrix
 * ┌            ┐
 * │ 1  0  0  0 │
 * │ 0  1  0  0 │
 * │ 0  0  1  0 │
 * │ 0  0  0  1 │
 * └            ┘
 */
Matrix4x4::Matrix4x4() {
  for (size_t i = 0; i < SIZE; ++i) {
    for (size_t j = 0; j < SIZE; ++j) {
      m[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
}

// Constructor from data
Matrix4x4::Matrix4x4(const std::array<std::array<double, SIZE>, SIZE>& data)
    : m(data) {}

// Identity factory method
Matrix4x4 Matrix4x4::identity() {
  return Matrix4x4();  // Default constructor makes identity
}

// Element getter
double Matrix4x4::get(size_t row, size_t col) const {
  if (row >= SIZE || col >= SIZE) {
    throw std::out_of_range("Invalid matrix index\n");
  }
  return m[row][col];
}

// Element Setter
void Matrix4x4::set(size_t row, size_t col, double value) {
  if (row >= SIZE || col >= SIZE) {
    throw std::out_of_range("Invalid matrix index\n");
  }
  m[row][col] = value;
}

// Matrix printer, if needed for debugging
void Matrix4x4::print() const {
  printf("\n");
  for (size_t i{0}; i < SIZE; i++) {
    printf("| ");
    for (size_t j{0}; j < SIZE; j++) {
      printf("%0.2f ", m[i][j]);
    }
    printf("|\n");
  }
  printf("\n");
}

/* Creates Translation Matrix
 * ┌             ┐
 * │ 1  0  0  Tx │
 * │ 0  1  0  Ty │
 * │ 0  0  1  Tz │
 * │ 0  0  0  1  │
 * └             ┘
 */
Matrix4x4 Matrix4x4::translation(double tran_x, double tran_y, double tran_z) {
  Matrix4x4 result;  // Starts as identity

  result.set(T_X, T_COL, tran_x);
  result.set(T_Y, T_COL, tran_y);
  result.set(T_Z, T_COL, tran_z);

  return result;
}

/* Creates Matrix for Rx
 * ┌                      ┐
 * │ 1    0       0     0 │
 * │ 0  cos(θ) -sin(θ)  0 │
 * │ 0  sin(θ)  cos(θ)  0 │
 * │ 0    0       0     1 │
 * └                      ┘
 */
Matrix4x4 Matrix4x4::rotationX(double angle_rad) {
  Matrix4x4 result;  // Starts as identity

  result.set(1, 1, std::cos(angle_rad));
  result.set(1, 2, -1 * std::sin(angle_rad));
  result.set(2, 1, std::sin(angle_rad));
  result.set(2, 2, std::cos(angle_rad));

  return result;
}

/* Creates Matrix for Ry
 * ┌                       ┐
 * │  cos(θ)  0  sin(θ)  0 │
 * │    0     1    0     0 │
 * │ -sin(θ)  0  cos(θ)  0 │
 * │    0     0    0     1 │
 * └                       ┘
 */
Matrix4x4 Matrix4x4::rotationY(double angle_rad) {
  Matrix4x4 result;  // Starts as identity

  result.set(0, 0, std::cos(angle_rad));
  result.set(0, 2, std::sin(angle_rad));
  result.set(2, 0, -1 * std::sin(angle_rad));
  result.set(2, 2, std::cos(angle_rad));

  return result;
}

/* Creates Matrix for Rz
 * ┌                       ┐
 * │ cos(θ) -sin(θ)  0   0 │
 * │ sin(θ)  cos(θ)  0   0 │
 * │   0       0     1   0 │
 * │   0       0     0   1 │
 * └                       ┘
 */
Matrix4x4 Matrix4x4::rotationZ(double angle_rad) {
  Matrix4x4 result;  // Starts as identity

  result.set(0, 0, std::cos(angle_rad));
  result.set(0, 1, -1 * std::sin(angle_rad));
  result.set(1, 0, std::sin(angle_rad));
  result.set(1, 1, std::cos(angle_rad));

  return result;
}

// Matrix Multiplication
Matrix4x4 Matrix4x4::operator*(const Matrix4x4& other) const {
  Matrix4x4 result;
  for (size_t row{0}; row < SIZE; row++) {
    for (size_t col{0}; col < SIZE; col++) {
      // Prepare summation before loop
      double sum{0};

      //           n-1
      // C[i][j] = ∑ A[i][k] * B[k][j]
      //           k=0
      for (size_t sum_enum{0}; sum_enum < SIZE; sum_enum++) {
        sum += (this->get(row, sum_enum) * other.get(sum_enum, col));
      }

      // Apply summation
      result.set(row, col, sum);
    }
  }
  return result;
};

// Chain Multiplication
Matrix4x4& Matrix4x4::operator*=(const Matrix4x4& other) {
  Matrix4x4 temp = *this * other;
  *this = std::move(temp);
  return *this;
}

// Comparison
bool Matrix4x4::equals(const Matrix4x4& other, double tolerance) const {
  // If the matrix is exactly the same, return true
  if (this == &other) {
    return true;
  }

  // Otherwise, check all the elements with tolerance
  for (size_t i{0}; i < SIZE; i++) {
    for (size_t j{0}; j < SIZE; j++) {
      // If difference in elements are larger than tolerance, 
      //  return false
      if (fabs(this->get(i, j) - other.get(i, j)) > tolerance) {
        return false;
      }
    }
  }
  return true;
};

// Creates Vector from translation coordinates of matrix
Vector3D Matrix4x4::getTranslation() const {
  return Vector3D{m[T_X][T_COL], m[T_Y][T_COL], m[T_Z][T_COL]};
};

}  // namespace math
}  // namespace arm
