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

#include <cmath>
#include <iomanip>
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
    throw std::out_of_range("Invalid matrix index");
  }
  return m[row][col];
};

// Element Setter
void Matrix4x4::set(size_t row, size_t col, double value) {
  if (row >= SIZE || col >= SIZE) {
    throw std::out_of_range("Invalid matrix index");
  }
  m[row][col] = value;
};

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
};

Matrix4x4 Matrix4x4::translation(double tran_x, double tran_y, double tran_z) {
  Matrix4x4 result;  // Starts as identity

  result.set(0, 3, tran_x);
  result.set(1, 3, tran_y);
  result.set(2, 3, tran_z);

  return result;
}

Matrix4x4 Matrix4x4::rotationX(double angle_rad) {
  Matrix4x4 result;  // Starts as identity

  result.set(1, 1, std::cos(angle_rad));
  result.set(1, 2, -1*std::sin(angle_rad));
  result.set(2, 1, std::sin(angle_rad));
  result.set(2, 2, std::cos(angle_rad));

  return result;
}

Matrix4x4 Matrix4x4::rotationY(double angle_rad) {
  Matrix4x4 result;  // Starts as identity

  result.set(0, 0, std::cos(angle_rad));
  result.set(0, 2, std::sin(angle_rad));
  result.set(2, 0, -1*std::sin(angle_rad));
  result.set(2, 2, std::cos(angle_rad));

  return result;
}

Matrix4x4 Matrix4x4::rotationZ(double angle_rad) {
  Matrix4x4 result;  // Starts as identity

  result.set(0, 0, std::cos(angle_rad));
  result.set(0, 1, -1*std::sin(angle_rad));
  result.set(1, 0, std::sin(angle_rad));
  result.set(1, 1, std::cos(angle_rad));

  return result;
}

}  // namespace math
}  // namespace arm