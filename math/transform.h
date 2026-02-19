/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-18 22:18:30
 * @modify date 2026-02-18 22:18:30
 * @desc Header for Transform class
 *       Wrapper used for matrix math
 */
#pragma once

#include "matrix4x4.h"

namespace arm {
namespace math {

class Transform {
private:
  Matrix4x4 matrix;  // Wraps a Matrix4x4

public:
  explicit Transform(const Matrix4x4& mat = Matrix4x4::identity());
  ~Transform() = default;
  
  // Factory methods
  static Transform identity();
  static Transform translation(double x, double y, double z);
  static Transform rotationX(double angle_rad);
  static Transform rotationY(double angle_rad);
  static Transform rotationZ(double angle_rad);
  
  // Chainable methods
  Transform& translate(double x, double y, double z);
  Transform& rotateX(double rad);
  Transform& rotateY(double rad);
  Transform& rotateZ(double rad);
  
  // Accessors
  Vector3D getTranslation() const { return matrix.getTranslation();};
  Matrix4x4 getMatrix() const { return matrix; }
  
  // Operations
  Transform operator*(const Transform& other) const;
  Transform& operator*=(const Transform& other);

  // Comparison
  bool equals(const Transform& other, double tolerance = 1e-6) const;
};

}  // namespace math
}  // namespace arm