/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-17 19:25:36
 * @modify date 2026-02-17 19:25:36
 * @desc Header for Matrix4x4
 *       Used for conducting 4x4 math
 *       for calculating arm joint rotations/translations
 */
#pragma once

#include <array>
#include <cmath>

#include "vector3d.h"

namespace arm {
namespace math {

class Matrix4x4 {
 private:
  static constexpr size_t SIZE = 4;

  // Translation coordinates
  static constexpr size_t T_COL = 3;
  static constexpr size_t T_X = 0;
  static constexpr size_t T_Y = 1;
  static constexpr size_t T_Z = 2;

 protected:
  std::array<std::array<double, SIZE>, SIZE> m;  // 4x4 array

 public:
  // Constructors
  Matrix4x4();  // Identity matrix by default
  Matrix4x4(const std::array<std::array<double, SIZE>, SIZE>& data);
  ~Matrix4x4() = default;

  // Factory methods for common transformations
  static Matrix4x4 identity();
  static Matrix4x4 translation(double x, double y, double z);
  static Matrix4x4 rotationX(double angle_rad);
  static Matrix4x4 rotationY(double angle_rad);
  static Matrix4x4 rotationZ(double angle_rad);

  // Matrix operations
  Matrix4x4 operator*(const Matrix4x4& other) const;
  Matrix4x4& operator*=(const Matrix4x4& other);

  // Element access
  double get(size_t row, size_t col) const;
  void set(size_t row, size_t col, double value);

  // Utility
  Vector3D getTranslation() const;  // Extract position from matrix
  void print() const;               // For debugging

  // Comparison
  bool equals(const Matrix4x4& other, double tolerance = 1e-6) const;

  // Getter
  std::array<std::array<double, SIZE>, SIZE> get_data() const { return m; };
};

}  // namespace math
}  // namespace arm