/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-17 19:25:36
 * @modify date 2026-02-17 19:25:36
 * @desc Header for Transform
 *       Used to conduct se3 rigid
 *       body transformations, containing
 *       homogenous rotational and translational
 *       coordinates.
 */
#pragma once

#include <array>
#include <cmath>

#include "vector3d.h"

namespace arm {
namespace math {

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
class Transform {
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
  Transform();  // Identity matrix by default
  Transform(const std::array<std::array<double, SIZE>, SIZE>& data);
  ~Transform() = default;

  // Factory methods for common transformations
  static Transform identity();
  static Transform translation(double x, double y, double z);
  static Transform rotationX(double angle_rad);
  static Transform rotationY(double angle_rad);
  static Transform rotationZ(double angle_rad);

  // Fluent chaining
  Transform& translate(double x, double y, double z);
  Transform& rotateX(double rad);
  Transform& rotateY(double rad);
  Transform& rotateZ(double rad);

  // Matrix operations
  Transform operator*(const Transform& other) const;
  Transform& operator*=(const Transform& other);

  // Accessors
  double get(size_t row, size_t col) const;
  void set(size_t row, size_t col, double value);
  Vector3D getTranslation() const;  // Extract position from matrix
  std::array<double, 3> getRotColumn(size_t idx) const;
  std::array<std::array<double, SIZE>, SIZE> get_data() const { return m; };

  // Utility
  bool equals(const Transform& other, double tolerance = 1e-6) const;
  void print() const;  // For debugging
};

}  // namespace math
}  // namespace arm
