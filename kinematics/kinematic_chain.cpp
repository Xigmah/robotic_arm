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
math::Vector3D KinematicChain::computeForwardKinematics() const {
  // Joint indices
  size_t j0{0};
  size_t j1{1};
  size_t j2{2};

  math::Transform T;

  /* Each transform takes original matrix, and multiplies by
   * requested transformation, therefore each rotation/translation
   * can be applied to the same Transform matrix
   */
  // Joint 0 - Base Rotation
  T.rotateZ(arm_config.getJointAngle(j0))
      .translate(arm_config.getLinkLength(j0), 0.0, 0.0);

  // Joint 1 - Shoulder
  T.rotateY(arm_config.getJointAngle(j1))
      .translate(arm_config.getLinkLength(j1), 0.0, 0.0);

  // Joint 2 - Elbow
  T.rotateY(arm_config.getJointAngle(j2))
      .translate(arm_config.getLinkLength(j2), 0.0, 0.0);

  return T.getTranslation();
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
