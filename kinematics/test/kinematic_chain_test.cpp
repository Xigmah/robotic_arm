#include "../kinematic_chain.h"

#include <gtest/gtest.h>

#include "../../hardware/joint_factory.h"
#include "../../math/math_util.h"

using namespace arm::kinematics;
using namespace arm::hardware;
using namespace arm::math;

class KinematicChainTest : public ::testing::Test {
 protected:
  ArmConfig config;

  void SetUp() override {
    config.addJoint(JointFactory::create(JointType::SIMULATED, deg_to_rad(-180),
                                         deg_to_rad(180), 100, RotAxis::Z));
    config.addJoint(JointFactory::create(JointType::SIMULATED, deg_to_rad(-180),
                                         deg_to_rad(180), 80, RotAxis::Y));
    config.addJoint(JointFactory::create(JointType::SIMULATED, deg_to_rad(-90),
                                         deg_to_rad(90), 60, RotAxis::Y));
  }
};

TEST_F(KinematicChainTest, testNumJoints) {
  KinematicChain chain(config);
  EXPECT_EQ(chain.getNumJoints(), 3);
}

TEST_F(KinematicChainTest, testSetAngle) {
  KinematicChain chain(config);

  std::vector<double> angles{0.3, 0.5, 0.8, 1.0};
  EXPECT_THROW(chain.setAllJointAngles(angles), std::runtime_error);

  angles = {0.3, 0.5, 0.8};
  chain.setAllJointAngles(angles);
  EXPECT_EQ(chain.getAllJointAngles(), angles);

  angles[1] = 1.0;
  chain.setJointAngle(1, 1.0);
  EXPECT_EQ(chain.getAllJointAngles(), angles);
}

TEST_F(KinematicChainTest, testFKZeroRot) {
  KinematicChain chain(config);
  auto result = chain.computFKTranslation();

  EXPECT_NEAR(result.get_x(), 240.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 0.0, 1e-6);
}

TEST_F(KinematicChainTest, testFKBaseRot) {
  KinematicChain chain(config);

  chain.setJointAngle(0, deg_to_rad(90));
  auto result = chain.computFKTranslation();
  EXPECT_NEAR(result.get_x(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 240.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 0.0, 1e-6);

  chain.setJointAngle(0, deg_to_rad(-90));
  result = chain.computFKTranslation();
  EXPECT_NEAR(result.get_x(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_y(), -240.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 0.0, 1e-6);

  chain.setJointAngle(0, deg_to_rad(180));
  result = chain.computFKTranslation();
  EXPECT_NEAR(result.get_x(), -240.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 0.0, 1e-6);
}

TEST_F(KinematicChainTest, testFKShoulderRot) {
  KinematicChain chain(config);

  chain.setJointAngle(1, deg_to_rad(90));
  auto result = chain.computFKTranslation();
  EXPECT_NEAR(result.get_x(), 100.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), -140.0, 1e-6);

  chain.setJointAngle(1, deg_to_rad(-90));
  result = chain.computFKTranslation();
  EXPECT_NEAR(result.get_x(), 100.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 140.0, 1e-6);

  chain.setJointAngle(1, deg_to_rad(180));
  result = chain.computFKTranslation();
  EXPECT_NEAR(result.get_x(), -40.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), 0.0, 1e-6);
}

TEST_F(KinematicChainTest, testFKElbowRot) {
  KinematicChain chain(config);

  chain.setJointAngle(2, deg_to_rad(90));
  auto result = chain.computFKTranslation();
  EXPECT_NEAR(result.get_x(), 180.0, 1e-6);
  EXPECT_NEAR(result.get_y(), 0.0, 1e-6);
  EXPECT_NEAR(result.get_z(), -60.0, 1e-6);
}

TEST_F(KinematicChainTest, testFKCombinedRotations) {
  KinematicChain chain(config);

  chain.setJointAngle(0, deg_to_rad(45));
  chain.setJointAngle(1, deg_to_rad(45));
  auto result = chain.computFKTranslation();

  EXPECT_GT(result.get_x(), 0.0);
  EXPECT_GT(result.get_y(), 0.0);
  EXPECT_LT(result.get_z(), 0.0);
}

TEST_F(KinematicChainTest, testFKAtJointLimits) {
  KinematicChain chain(config);

  chain.setJointAngle(0, deg_to_rad(180));
  chain.setJointAngle(1, deg_to_rad(90));
  chain.setJointAngle(2, deg_to_rad(90));
  auto result = chain.computFKTranslation();

  EXPECT_FALSE(std::isnan(result.get_x()));
  EXPECT_FALSE(std::isnan(result.get_y()));
  EXPECT_FALSE(std::isnan(result.get_z()));
}
