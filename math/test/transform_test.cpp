#include "transform.h"

#include <gtest/gtest.h>

#include <cstring>

using namespace arm::math;

TEST(TransformTests, DefaultConstructorIsIdentity) {
  Transform t;
  Vector3D pos = t.getTranslation();
  
  EXPECT_EQ(pos.get_x(), 0.0);
  EXPECT_EQ(pos.get_y(), 0.0);
  EXPECT_EQ(pos.get_z(), 0.0);
}

TEST(TransformTests, ChainedTransformations) {
  Transform t = Transform()
      .translate(10, 0, 0)
      .rotateZ(M_PI / 2)
      .translate(5, 0, 0);
  
  Vector3D pos = t.getTranslation();
  
  // After rotation, second translation goes in different direction
  EXPECT_NEAR(pos.get_x(), 10.0, 1e-6);
  EXPECT_NEAR(pos.get_y(), 5.0, 1e-6);
}

TEST(TransformTests, MultiplyTransforms) {
  Transform t1 = Transform::translation(10, 0, 0);
  Transform t2 = Transform::translation(5, 0, 0);
  
  Transform result = t1 * t2;
  Vector3D pos = result.getTranslation();
  
  EXPECT_NEAR(pos.get_x(), 15.0, 1e-6);
}

TEST(TransformTests, FactoryMethods) {
  Transform t = Transform::rotationZ(M_PI / 4);
  
  // Should have rotation but no translation
  Vector3D pos = t.getTranslation();
  EXPECT_EQ(pos.get_x(), 0.0);
  EXPECT_EQ(pos.get_y(), 0.0);
}