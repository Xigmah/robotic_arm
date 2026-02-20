/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-16 16:50:51
 * @modify date 2026-02-16 16:50:51
 * @desc Code for Vector3D
 *       Used to calculate vectors
 *       in a 3D space
 */

#include "vector3d.h"

#include <cmath>

namespace arm {
namespace math {

// Default constructor
Vector3D::Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

// Addition
Vector3D Vector3D::operator+(const Vector3D& other) const {
  return Vector3D{x + other.x, y + other.y, z + other.z};
}

// Subtraction
Vector3D Vector3D::operator-(const Vector3D& other) const {
  return Vector3D{x - other.x, y - other.y, z - other.z};
}

// Scalar multiplication
Vector3D Vector3D::operator*(double scalar) const {
  return Vector3D{x * scalar, y * scalar, z * scalar};
}

// Magnitude (length of vector)
double Vector3D::magnitude() const { return std::sqrt(x * x + y * y + z * z); }

// Distance between two points
double Vector3D::distance(const Vector3D& other) const {
  Vector3D temp = *this - other;
  return temp.magnitude();
}

// Normalize (unit vector)
Vector3D Vector3D::normalized() const {
  double mag = magnitude();
  if (mag < 1e-10) {  // Very small, can't normalize
    return Vector3D(0, 0, 0);
  }
  return *this * (1 / mag);
}

// Dot product
double Vector3D::dot(const Vector3D& other) const {
  return (x * other.x + y * other.y + z * other.z);
}

// Equality with tolerance (important for floating point!)
bool Vector3D::equals(const Vector3D& other, double tolerance) const {
  return distance(other) < tolerance;
}

}  // namespace math
}  // namespace arm
