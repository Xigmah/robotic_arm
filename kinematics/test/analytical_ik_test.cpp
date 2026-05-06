#include "../analytical_ik.h"

#include <gtest/gtest.h>

#include "../../hardware/joint_factory.h"
#include "../kinematic_chain.h"

using namespace arm::kinematics;
using namespace arm::hardware;
using namespace arm::math;

class AnalyticIKTests : public ::testing::Test {
 protected:
  ArmConfig config;

  void SetUp() override {
    config.addJoint(JointFactory::create(JointType::SIMULATED, -M_PI, M_PI, 100,
                                         RotAxis::Z));
    config.addJoint(JointFactory::create(JointType::SIMULATED, -M_PI_2, M_PI_2,
                                         80, RotAxis::Y));
    config.addJoint(JointFactory::create(JointType::SIMULATED, -M_PI_2, M_PI_2,
                                         60, RotAxis::Y));
  }
};

// Test full solve() with FK verification
TEST_F(AnalyticIKTests, Solve_VerifyWithFK_AlongXAxis) {
  KinematicChain chain(config);
  AnalyticIK3DOF ik(config);

  Vector3D target(200, 0, 0);

  auto angles = ik.solve(target);
  ASSERT_TRUE(angles.has_value());

  chain.setAllJointAngles(*angles);
  auto result = chain.computFKTranslation();

  EXPECT_NEAR(result.get_x(), target.get_x(), 1e-3);
  EXPECT_NEAR(result.get_y(), target.get_y(), 1e-3);
  EXPECT_NEAR(result.get_z(), target.get_z(), 1e-3);
}

TEST_F(AnalyticIKTests, Solve_VerifyWithFK_Diagonal) {
  KinematicChain chain(config);
  AnalyticIK3DOF ik(config);

  Vector3D target(150, 150, 0);

  auto angles = ik.solve(target);
  ASSERT_TRUE(angles.has_value());

  chain.setAllJointAngles(*angles);
  auto result = chain.computFKTranslation();

  EXPECT_NEAR(result.get_x(), target.get_x(), 1e-3);
  EXPECT_NEAR(result.get_y(), target.get_y(), 1e-3);
  EXPECT_NEAR(result.get_z(), target.get_z(), 1e-3);
}

TEST_F(AnalyticIKTests, Solve_VerifyWithFK_WithHeight) {
  KinematicChain chain(config);
  AnalyticIK3DOF ik(config);

  Vector3D target(170, 100, 40);

  auto angles = ik.solve(target);
  ASSERT_TRUE(angles.has_value());

  chain.setAllJointAngles(*angles);
  auto result = chain.computFKTranslation();

  EXPECT_NEAR(result.get_x(), target.get_x(), 1e-3);
  EXPECT_NEAR(result.get_y(), target.get_y(), 1e-3);
  EXPECT_NEAR(result.get_z(), target.get_z(), 1e-3);
}

TEST_F(AnalyticIKTests, Solve_VerifyWithFK_NegativeZ) {
  KinematicChain chain(config);
  AnalyticIK3DOF ik(config);

  Vector3D target(150, 150, -30);

  auto angles = ik.solve(target);
  ASSERT_TRUE(angles.has_value());

  chain.setAllJointAngles(*angles);
  auto result = chain.computFKTranslation();

  EXPECT_NEAR(result.get_x(), target.get_x(), 1e-3);
  EXPECT_NEAR(result.get_y(), target.get_y(), 1e-3);
  EXPECT_NEAR(result.get_z(), target.get_z(), 1e-3);
}

TEST_F(AnalyticIKTests, Solve_MaxReach) {
  KinematicChain chain(config);
  AnalyticIK3DOF ik(config);

  // Maximum reach: 100 + 80 + 60 = 240
  Vector3D target(240, 0, 0);

  auto angles = ik.solve(target);
  ASSERT_TRUE(angles.has_value());

  chain.setAllJointAngles(*angles);
  auto result = chain.computFKTranslation();

  EXPECT_NEAR(result.get_x(), target.get_x(), 1e-3);
}

TEST_F(AnalyticIKTests, Solve_BeyondMaxReach) {
  AnalyticIK3DOF ik(config);

  Vector3D target(250, 0, 0);  // Beyond 240mm max reach

  auto angles = ik.solve(target);
  EXPECT_FALSE(angles.has_value());
}

TEST_F(AnalyticIKTests, Solve_AtOrigin) {
  AnalyticIK3DOF ik(config);

  Vector3D target(0, 0, 0);  // At base origin

  auto angles = ik.solve(target);
  EXPECT_FALSE(angles.has_value());  // Should be unreachable (inside base link)
}

TEST_F(AnalyticIKTests, Solve_MinimumReach) {
  KinematicChain chain(config);
  AnalyticIK3DOF ik(config);

  // Minimum reach: typically |L1 - L2 - L3| = |100 - 80 - 60| = -40
  // But actually depends on configuration
  Vector3D target(120, 0, 0);  // Close but should be reachable

  auto angles = ik.solve(target);
  if (angles.has_value()) {
    chain.setAllJointAngles(*angles);
    auto result = chain.computFKTranslation();

    EXPECT_NEAR(result.get_x(), target.get_x(), 1e-3);
  }
}

TEST_F(AnalyticIKTests, Solve_AllQuadrants) {
  KinematicChain chain(config);
  AnalyticIK3DOF ik(config);

  // Test targets in all four quadrants
  std::vector<Vector3D> targets = {
      Vector3D(150, 150, 20),    // Q1
      Vector3D(-150, 150, 20),   // Q2
      Vector3D(-150, -150, 20),  // Q3
      Vector3D(150, -150, 20)    // Q4
  };

  for (const auto& target : targets) {
    auto angles = ik.solve(target);
    ASSERT_TRUE(angles.has_value())
        << "Failed for target (" << target.get_x() << ", " << target.get_y()
        << ", " << target.get_z() << ")";

    chain.setAllJointAngles(*angles);
    auto result = chain.computFKTranslation();

    EXPECT_NEAR(result.get_x(), target.get_x(), 1e-3);
    EXPECT_NEAR(result.get_y(), target.get_y(), 1e-3);
    EXPECT_NEAR(result.get_z(), target.get_z(), 1e-3);
  }
}

// Test isReachable
TEST_F(AnalyticIKTests, IsReachable_ValidTarget) {
  AnalyticIK3DOF ik(config);

  Vector3D target(150, 150, 0);
  EXPECT_TRUE(ik.isReachable(target));
}

TEST_F(AnalyticIKTests, IsReachable_UnreachableTarget) {
  AnalyticIK3DOF ik(config);

  Vector3D target(300, 300, 0);  // Way beyond reach
  EXPECT_FALSE(ik.isReachable(target));
}
