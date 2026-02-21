#include "../kinematic_chain.h"

#include <gtest/gtest.h>

#include "../../hardware/joint_factory.h"

/*
 * Ready to Test?
Want to write tests for FK? Here's a test header structure if you want to try:
Test cases to consider:

All joints at 0° → end effector at (L1+L2+L3, 0, 0)
Base rotated 90° → end effector should rotate around Z
Shoulder/elbow bent → verify position calculations
 */

using namespace arm::kinematics;
using namespace arm::hardware;

class KinematicChainTest : public ::testing::Test {
 protected:
  ArmConfig config;

  void SetUp() override {
    config.addJoint(JointFactory::create(JointType::SIMULATED, -180, 180, 100));
    config.addJoint(JointFactory::create(JointType::SIMULATED, -90, 90, 80));
    config.addJoint(JointFactory::create(JointType::SIMULATED, -90, 90, 60));
  }
};

TEST_F(KinematicChainTest, testNumJoints) {
  KinematicChain chain(config);

  EXPECT_EQ(chain.getNumJoints(), 3);
}

TEST_F(KinematicChainTest, testSetAngle) {
  KinematicChain chain(config);

  // Test runtime_error
  std::vector<double> angles{10.0, 20.0, 30.0, 40.0};
  EXPECT_THROW(chain.setAllJointAngles(angles), std::runtime_error);

  // Test set all angles
  angles = {10.0, 20.0, 30.0};
  chain.setAllJointAngles(angles);
  EXPECT_EQ(chain.getAllJointAngles(), angles);

  angles[1] = 40.0;
  chain.setJointAngle(1, 40.0);
  EXPECT_EQ(chain.getAllJointAngles(), angles);
}

TEST_F(KinematicChainTest, testFKZeroRot) {
  KinematicChain chain(config);
  auto result = chain.computeForwardKinematics();


  // Test zero rotation
  EXPECT_NEAR(result.get_x(), 240.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 0.0, 1e-6);
}

TEST_F(KinematicChainTest, testFKBaseRot) {
  KinematicChain chain(config);

  // Test base 90deg rotation
  chain.setJointAngle(0, M_PI_2);
  auto result = chain.computeForwardKinematics();

  EXPECT_NEAR(result.get_x(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 240.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 0.0, 1e-6);

  // Test base -90deg rotation
  chain.setJointAngle(0, -M_PI_2);
  result = chain.computeForwardKinematics();

  EXPECT_NEAR(result.get_x(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_y(), -240.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 0.0, 1e-6);

  // Test base 180deg rotation
  chain.setJointAngle(0, M_PI);
  result = chain.computeForwardKinematics();

  EXPECT_NEAR(result.get_x(), -240.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 0.0, 1e-6);
}

TEST_F(KinematicChainTest, testFKShoulderRot) {
  KinematicChain chain(config);

  // Test shoulder 90deg rotation
  chain.setJointAngle(1, M_PI_2);
  auto result = chain.computeForwardKinematics();

  EXPECT_NEAR(result.get_x(), 100.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), -140.0, 1e-6);

  // Test shoulder -90deg rotation
  chain.setJointAngle(1, -M_PI_2);
  result = chain.computeForwardKinematics();

  EXPECT_NEAR(result.get_x(), 100.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 140.0, 1e-6);

  // Test shoulder 180deg rotation
  chain.setJointAngle(1, M_PI);
  result = chain.computeForwardKinematics();

  EXPECT_NEAR(result.get_x(), -40.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 0.0, 1e-6);
}

TEST_F(KinematicChainTest, testFKElbowRot) {
  KinematicChain chain(config);
  
  // Elbow bent 90° (should behave like shoulder but further out)
  chain.setJointAngle(2, M_PI_2);
  auto result = chain.computeForwardKinematics();
  
  // Base: 100mm along X
  // Shoulder: 80mm along X (total 180mm)
  // Elbow rotates 90°, so 60mm goes along -Z
  EXPECT_NEAR(result.get_x(), 180.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), -60.0, 1e-6);
}

TEST_F(KinematicChainTest, testFKCombinedRotations) {
  KinematicChain chain(config);
  
  // Base 45°, shoulder 45°
  chain.setJointAngle(0, M_PI_4);  // 45° around Z
  chain.setJointAngle(1, M_PI_4);  // 45° around Y
  
  auto result = chain.computeForwardKinematics();
  
  // This is harder to calculate by hand, but verifies the chain works
  // At minimum, check it doesn't crash and produces reasonable values
  EXPECT_GT(result.get_x(), 0.0);  // Should still be positive X
  EXPECT_GT(result.get_y(), 0.0);  // Should have Y component from base rotation
  EXPECT_LT(result.get_z(), 0.0);  // Should dip down from shoulder rotation
}

TEST_F(KinematicChainTest, testFKAtJointLimits) {
  KinematicChain chain(config);
  
  // Set joints to their limits
  chain.setJointAngle(0, M_PI);   // Max base rotation
  chain.setJointAngle(1, M_PI_2);    // Max shoulder
  chain.setJointAngle(2, M_PI_2);    // Max elbow
  
  auto result = chain.computeForwardKinematics();
  
  // Should not crash, should produce valid position
  EXPECT_FALSE(std::isnan(result.get_x()));
  EXPECT_FALSE(std::isnan(result.get_y()));
  EXPECT_FALSE(std::isnan(result.get_z()));
}
