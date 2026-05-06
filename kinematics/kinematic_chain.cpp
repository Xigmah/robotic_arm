/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-20 10:59:07
 * @modify date 2026-02-20 10:59:07
 * @desc Code for Kinematic Chain
 *       Stores ArmConfig, and manages
 *       forward kinematics
 */
#include "kinematic_chain.h"

#include <stdexcept>

namespace arm {
namespace kinematics {

KinematicChain::KinematicChain(hardware::ArmConfig& config)
    : arm_config{config} {}

/* Compute end effector position from current joint angles
 * For 3-DOF arm (Z-Y-Y configuration):
 *
 * 1. Joint 0 (Base): Rotates around Z, has link length L1
 * 2. Joint 1 (Shoulder): Rotates around Y, has link length L2
 * 3. Joint 2 (Elbow): Rotates around Y, has link length L3
 *
 * Transform chain:
 * T_final = T0 * T1 * T2
 *
 * Where each `T_i` is:
 * T_i = Rotation(axis, angle) * Translation(link_length, 0, 0)
 */
ChainState KinematicChain::computeForwardKinematics(
    const hardware::ArmConfig& config) const {
  math::Transform T;

  ChainState JointFrames;

  /* Each transform takes original matrix, and multiplies by
   * requested transformation, therefore each rotation/translation
   * can be applied to the same Transform matrix
   */
  for (size_t idx{0}; idx < config.getNumJoints(); idx++) {
    // First, get joint frame data
    math::Vector3D joint_axis{0, 0, 0};
    switch (config.getJointRotAxis(idx)) {
      case hardware::RotAxis::X:
        joint_axis = T.getRotColumn(0);
        break;
      case hardware::RotAxis::Y:
        joint_axis = T.getRotColumn(1);
        break;
      case hardware::RotAxis::Z:
        joint_axis = T.getRotColumn(2);
        break;
      default:
        throw std::runtime_error("Unkown joint rotation\n");
    }

    // Store joint position and rotational axis prior to translation 
    JointFrame frame{T.getTranslation(), joint_axis};
    JointFrames.joint_frames.push_back(frame);

    // Then, do rotations and translations
    switch (config.getJointRotAxis(idx)) {
      case hardware::RotAxis::X:
        T.rotateX(config.getJointAngle(idx));
        break;
      case hardware::RotAxis::Y:
        T.rotateY(config.getJointAngle(idx));
        break;
      case hardware::RotAxis::Z:
        T.rotateZ(config.getJointAngle(idx));
        break;
      default:
        throw std::runtime_error("Unkown joint rotation\n");
    }
    T.translate(config.getLinkLength(idx), 0.0, 0.0);
  }
  JointFrames.end_eff_vec = T.getTranslation();

  return JointFrames;
}

math::Vector3D KinematicChain::computFKTranslation(
    const hardware::ArmConfig& config) const {
  ChainState temp{computeForwardKinematics(config)};
  return temp.end_eff_vec;
}

// Overload to use default config
math::Vector3D KinematicChain::computFKTranslation() const {
  return computFKTranslation(arm_config);
}

// Convenience (delegates to hardware)
size_t KinematicChain::getNumJoints() const {
  return arm_config.getNumJoints();
}

// radians
void KinematicChain::setJointAngle(size_t index, double angle_rad) {
  arm_config.setJointAngle(index, angle_rad);
}

// radians
void KinematicChain::setAllJointAngles(const std::vector<double>& angles) {
  arm_config.setAllJointAngles(angles);
}

// radians
std::vector<double> KinematicChain::getAllJointAngles() const {
  return arm_config.getAllJointAngles();
}

}  // namespace kinematics
}  // namespace arm
