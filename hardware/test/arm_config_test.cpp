#include "../arm_config.h"

#include <gtest/gtest.h>

#include "../joint_factory.h"

using namespace arm::hardware;

TEST(ArmConfigTests, AddJoints) {
  ArmConfig config;

  config.addJoint(JointFactory::create(JointType::SIMULATED, -180, 180, 100));
  config.addJoint(JointFactory::create(JointType::SIMULATED, -90, 90, 80));
  config.addJoint(JointFactory::create(JointType::SIMULATED, -90, 90, 60));

  EXPECT_EQ(config.getNumJoints(), 3);
}

TEST(ArmConfigTests, GetLinkLengths) {
  ArmConfig config;
  config.addJoint(JointFactory::create(JointType::SIMULATED, -180, 180, 100));
  config.addJoint(JointFactory::create(JointType::SIMULATED, -90, 90, 80));
  config.addJoint(JointFactory::create(JointType::SIMULATED, -90, 90, 60));

  EXPECT_EQ(config.getLinkLength(0), 100.0);
  EXPECT_EQ(config.getLinkLength(1), 80.0);
  EXPECT_EQ(config.getLinkLength(2), 60.0);
}

TEST(ArmConfigTests, SetAndGetJointAngles) {
  ArmConfig config;
  config.addJoint(JointFactory::create(JointType::SIMULATED, -180, 180, 100));
  config.addJoint(JointFactory::create(JointType::SIMULATED, -90, 90, 80));

  config.setJointAngle(0, 45.0);
  config.setJointAngle(1, -30.0);

  EXPECT_EQ(config.getJointAngle(0), 45.0);
  EXPECT_EQ(config.getJointAngle(1), -30.0);
}

TEST(ArmConfigTests, SetAllJointAngles) {
  ArmConfig config;
  config.addJoint(JointFactory::create(JointType::SIMULATED, -180, 180, 100));
  config.addJoint(JointFactory::create(JointType::SIMULATED, -90, 90, 80));
  config.addJoint(JointFactory::create(JointType::SIMULATED, -90, 90, 60));

  std::vector<double> angles = {10.0, 20.0, 30.0};
  config.setAllJointAngles(angles);

  auto result = config.getAllJointAngles();
  EXPECT_EQ(result[0], 10.0);
  EXPECT_EQ(result[1], 20.0);
  EXPECT_EQ(result[2], 30.0);
}

TEST(ArmConfigTests, OutOfBoundsThrows) {
  ArmConfig config;
  config.addJoint(JointFactory::create(JointType::SIMULATED, -180, 180, 100));

  EXPECT_THROW(config.getJoint(5), std::out_of_range);
  EXPECT_THROW(config.setJointAngle(5, 0), std::out_of_range);
}