#include "vector3d.h"

#include <gtest/gtest.h>

using namespace arm::math;

TEST(Vector3DTests, testConstructor) {
  Vector3D v1;

  EXPECT_EQ(v1.get_x(), 0.0);
  EXPECT_EQ(v1.get_y(), 0.0);
  EXPECT_EQ(v1.get_z(), 0.0);

  Vector3D v2(1.0, 2.0, 3.0);

  EXPECT_EQ(v2.get_x(), 1.0);
  EXPECT_EQ(v2.get_y(), 2.0);
  EXPECT_EQ(v2.get_z(), 3.0);
}

TEST(Vector3DTests, testAddition) {
  Vector3D v1(1, 2, 3);
  Vector3D v2(4, 5, 6);
  Vector3D result = v1 + v2;

  EXPECT_EQ(result.get_x(), 5.0);
  EXPECT_EQ(result.get_y(), 7.0);
  EXPECT_EQ(result.get_z(), 9.0);
}

TEST(Vector3DTests, testSubtraction) {
  Vector3D v1(5, 7, 9);
  Vector3D v2(1, 2, 3);
  Vector3D result = v1 - v2;

  EXPECT_EQ(result.get_x(), 4.0);
  EXPECT_EQ(result.get_y(), 5.0);
  EXPECT_EQ(result.get_z(), 6.0);
}

TEST(Vector3DTests, testScalarMultiply) {
  Vector3D v(1, 2, 3);
  Vector3D result = v * 2.0;

  EXPECT_EQ(result.get_x(), 2.0);
  EXPECT_EQ(result.get_y(), 4.0);
  EXPECT_EQ(result.get_z(), 6.0);
}

TEST(Vector3DTests, testMagnitude) {
  Vector3D v(3, 4, 0);  // 3-4-5 triangle
  double mag = v.magnitude();

  EXPECT_NEAR(mag, 5.0, 1e-6);
}

TEST(Vector3DTests, testDistance) {
  Vector3D v1(0, 0, 0);
  Vector3D v2(3, 4, 0);
  double dist = v1.distance(v2);

  EXPECT_NEAR(dist, 5.0, 1e-6);
}

TEST(Vector3DTests, testNormalized) {
  Vector3D v(3, 4, 0);
  Vector3D unit = v.normalized();

  // Unit vector should have magnitude 1
  EXPECT_NEAR(unit.magnitude(), 1.0, 1e-6);

  // Components should be (3/5, 4/5, 0)
  EXPECT_NEAR(unit.get_x(), 0.6, 1e-6);
  EXPECT_NEAR(unit.get_y(), 0.8, 1e-6);
}

TEST(Vector3DTests, testDotProduct) {
  Vector3D v1(1, 0, 0);
  Vector3D v2(0, 1, 0);
  double dot = v1.dot(v2);

  // Perpendicular vectors have dot product 0
  EXPECT_NEAR(dot, 0.0, 1e-6);

  Vector3D v3(1, 2, 3);
  Vector3D v4(4, 5, 6);
  double dot2 = v3.dot(v4);
  // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
  EXPECT_NEAR(dot2, 32.0, 1e-6);
}
