/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-18 22:25:37
 * @modify date 2026-02-18 22:25:37
 * @desc Code for Transform class
 *       Wrapper used for matrix math
 */

#include "transform.h"

namespace arm {
namespace math {

Transform::Transform(const Matrix4x4& mat) : matrix{mat} {};

Transform Transform::identity() { return Transform(Matrix4x4::identity()); }

Transform Transform::translation(double x, double y, double z) {
  return Transform(Matrix4x4::translation(x, y, z));
}

Transform Transform::rotationX(double angle_rad) {
  return Transform(Matrix4x4::rotationX(angle_rad));
}

Transform Transform::rotationY(double angle_rad) {
  return Transform(Matrix4x4::rotationY(angle_rad));
}

Transform Transform::rotationZ(double angle_rad) {
  return Transform(Matrix4x4::rotationZ(angle_rad));
}

Transform& Transform::translate(double x, double y, double z) {
  matrix *= Matrix4x4::translation(x, y, z);
  return *this;
}

Transform& Transform::rotateX(double rad) {
  matrix *= Matrix4x4::rotationX(rad);
  return *this;
}

Transform& Transform::rotateY(double rad) {
  matrix *= Matrix4x4::rotationY(rad);
  return *this;
}

Transform& Transform::rotateZ(double rad) {
  matrix *= Matrix4x4::rotationZ(rad);
  return *this;
}

Transform Transform::operator*(const Transform& other) const {
  return Transform(matrix * other.matrix);
}

Transform& Transform::operator*=(const Transform& other) {
  matrix *= other.matrix;
  return *this;
}

bool Transform::equals(const Transform& other, double tolerance) const {
  return matrix.equals(other.matrix, tolerance);
}

}  // namespace math
}  // namespace arm
