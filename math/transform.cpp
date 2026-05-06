/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-17 19:35:10
 * @modify date 2026-02-17 19:35:10
 * @desc Code for Transform
 *       Used to conduct se3 rigid
 *       body transformations, containing
 *       homogenous rotational and translational
 *       coordinates.
 */

#include "transform.h"

#include <iostream>

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
Transform::Transform() {
  for (size_t i = 0; i < SIZE; ++i) {
    for (size_t j = 0; j < SIZE; ++j) {
      m[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
}

// Constructor from data
Transform::Transform(const std::array<std::array<double, SIZE>, SIZE>& data)
    : m(data) {}

// Identity factory method
Transform Transform::identity() {
  return Transform();  // Default constructor makes identity
}

// Element getter
double Transform::get(size_t row, size_t col) const {
  if (row >= SIZE || col >= SIZE) {
    throw std::out_of_range("Invalid matrix index\n");
  }
  return m[row][col];
}

// Element Setter
void Transform::set(size_t row, size_t col, double value) {
  if (row >= SIZE || col >= SIZE) {
    throw std::out_of_range("Invalid matrix index\n");
  }
  m[row][col] = value;
}

// Matrix printer, if needed for debugging
void Transform::print() const {
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
Transform Transform::translation(double tran_x, double tran_y, double tran_z) {
  Transform result;  // Starts as identity

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
Transform Transform::rotationX(double angle_rad) {
  Transform result;  // Starts as identity

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
Transform Transform::rotationY(double angle_rad) {
  Transform result;  // Starts as identity

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
Transform Transform::rotationZ(double angle_rad) {
  Transform result;  // Starts as identity

  result.set(0, 0, std::cos(angle_rad));
  result.set(0, 1, -1 * std::sin(angle_rad));
  result.set(1, 0, std::sin(angle_rad));
  result.set(1, 1, std::cos(angle_rad));

  return result;
}

// Matrix Multiplication
Transform Transform::operator*(const Transform& other) const {
  Transform result;
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
Transform& Transform::operator*=(const Transform& other) {
  Transform temp = *this * other;
  *this = std::move(temp);
  return *this;
}

// Comparison
bool Transform::equals(const Transform& other, double tolerance) const {
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
Vector3D Transform::getTranslation() const {
  return Vector3D{m[T_X][T_COL], m[T_Y][T_COL], m[T_Z][T_COL]};
};

Transform& Transform::translate(double x, double y, double z) {
  return *this *= Transform::translation(x, y, z);
}

Transform& Transform::rotateX(double rad) {
  return *this *= Transform::rotationX(rad);
}

Transform& Transform::rotateY(double rad) {
  return *this *= Transform::rotationY(rad);
}

Transform& Transform::rotateZ(double rad) {
  return *this *= Transform::rotationZ(rad);
}

std::array<double, 3> Transform::getRotColumn(size_t idx) const {
  if (idx > 2) {
    throw std::out_of_range("Selected column index out of range\n");
  }
  return std::array<double, 3>{this->get(0, idx), this->get(1, idx),
                               this->get(2, idx)};
}

}  // namespace math
}  // namespace arm
