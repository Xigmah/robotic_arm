#include <gtest/gtest.h>

#include "../simulated_joint.h"

using namespace arm::hardware;

TEST(JointTests, ConstructorInitialization) {
  SimulatedJoint joint(-90, 90, 100, RotAxis::Y);

  EXPECT_EQ(joint.getMinLimit(), -90.0);
  EXPECT_EQ(joint.getMaxLimit(), 90.0);
  EXPECT_EQ(joint.getLinkLength(), 100.0);
}

TEST(JointTests, SetAngleWithinLimits) {
  SimulatedJoint joint(-90, 90, 100, RotAxis::Y);

  joint.setTargetAngle(45.0);
  EXPECT_EQ(joint.getCurrentAngle(), 45.0);
}

TEST(JointTests, SetAngleClampToMin) {
  SimulatedJoint joint(-90, 90, 100, RotAxis::Y);

  joint.setTargetAngle(-120.0);               // Below limit
  EXPECT_EQ(joint.getCurrentAngle(), -90.0);  // Should clamp
}

TEST(JointTests, SetAngleClampToMax) {
  SimulatedJoint joint(-90, 90, 100, RotAxis::Y);

  joint.setTargetAngle(120.0);               // Above limit
  EXPECT_EQ(joint.getCurrentAngle(), 90.0);  // Should clamp
}

TEST(JointTests, IsWithinLimits) {
  SimulatedJoint joint(-90, 90, 100, RotAxis::Y);

  EXPECT_TRUE(joint.isWithinLimits(0.0));
  EXPECT_TRUE(joint.isWithinLimits(-90.0));
  EXPECT_TRUE(joint.isWithinLimits(90.0));
  EXPECT_FALSE(joint.isWithinLimits(-91.0));
  EXPECT_FALSE(joint.isWithinLimits(91.0));
}
