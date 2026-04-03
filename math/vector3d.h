/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-16 16:03:20
 * @modify date 2026-02-16 16:03:20
 * @desc Header for Vector3D
 *       Used to calculate vectors
 *       in a 3D space
 */
#pragma once

namespace arm {
namespace math {

class Vector3D {
 private:
  static constexpr double def_x{0.0};
  static constexpr double def_y{0.0};
  static constexpr double def_z{0.0};

 protected:
  double x, y, z;

 public:
  // Constructors
  Vector3D(double x = def_x, double y = def_y, double z = def_z);
  ~Vector3D() = default;

  // Operations
  Vector3D operator+(const Vector3D& other) const;  // v1 + v2
  Vector3D operator-(const Vector3D& other) const;  // v1 - v2
  Vector3D operator*(double scalar) const;          // v * 2.0

  // Utility
  double magnitude() const;                      // Length of vector
  double distance(const Vector3D& other) const;  // Distance between points
  double dot(const Vector3D& other) const;       // Dot product
  Vector3D normalized() const;                   // Unit vector
  Vector3D cross(const Vector3D& other) const;   // Cross Product

  // Comparison (with tolerance for floating point)
  bool equals(const Vector3D& other, double tolerance = 1e-6) const;

  // Getters
  double get_x() const { return x; };
  double get_y() const { return y; };
  double get_z() const { return z; };
};

}  // namespace math
}  // namespace arm
