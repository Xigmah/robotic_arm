#include "../jacobian_ik.h"

#include <gtest/gtest.h>

#include "../../hardware/joint_factory.h"
#include "../../math/math_util.h"
#include "../kinematic_chain.h"

using namespace arm::kinematics;
using namespace arm::hardware;
using namespace arm::math;

class JacobianIKTests : public ::testing::Test {
 protected:
  ArmConfig config;

  void SetUp() override {
    config.addJoint(JointFactory::create(JointType::SIMULATED, -M_PI, M_PI, 100,
                                         RotAxis::Z));
    config.addJoint(JointFactory::create(JointType::SIMULATED, -M_PI_2, M_PI_2,
                                         80, RotAxis::Y));
    config.addJoint(JointFactory::create(JointType::SIMULATED, -deg_to_rad(135),
                                         deg_to_rad(135), 60, RotAxis::Y));
    // config.addJoint(JointFactory::create(JointType::SIMULATED,
    // -deg_to_rad(135), deg_to_rad(135),
    //                                      60, RotAxis::Y));
    config.addJoint(JointFactory::create(JointType::SIMULATED, -M_PI, M_PI, 60,
                                         RotAxis::X));
  }
};

// Test full solve() with FK verification
TEST_F(JacobianIKTests, Solve_VerifyWithFK_AlongXAxis) {
  KinematicChain chain(config);
  JacobianIK ik(config);

  Vector3D target(300, 0, 0);

  auto angles = ik.solve(target);
  ASSERT_TRUE(angles.has_value());

  chain.setAllJointAngles(*angles);
  auto result = chain.computFKTranslation();

  EXPECT_NEAR(result.get_x(), target.get_x(), 1e-3);
  EXPECT_NEAR(result.get_y(), target.get_y(), 1e-3);
  EXPECT_NEAR(result.get_z(), target.get_z(), 1e-3);
}

// Test full solve() with target that necessitates joint bend
TEST_F(JacobianIKTests, Solve_TargetWithBend) {
  KinematicChain chain(config);
  JacobianIK ik(config);

  Vector3D target(250, 50, 0);

  auto angles = ik.solve(target);
  ASSERT_TRUE(angles.has_value());

  chain.setAllJointAngles(*angles);
  auto result = chain.computFKTranslation();

  EXPECT_NEAR(result.get_x(), target.get_x(), 1);
  EXPECT_NEAR(result.get_y(), target.get_y(), 1);
  EXPECT_NEAR(result.get_z(), target.get_z(), 1);
}
