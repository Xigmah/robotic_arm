/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-20 10:57:57
 * @modify date 2026-02-20 10:57:57
 * @desc Header for Kinematic Chain
 *       Stores ArmConfig, and manages
 *       forward kinematics
 */
#pragma once

#include "../hardware/arm_config.h"
#include "../math/transform.h"
#include "kinematics_util.h"

namespace arm {
namespace kinematics {

class KinematicChain {
 private:
  hardware::ArmConfig& arm_config;

 public:
  explicit KinematicChain(hardware::ArmConfig& config);
  ~KinematicChain() = default;

  // Compute end effector position from current joint angles
  math::Vector3D computFKTranslation() const;
  math::Vector3D computFKTranslation(const hardware::ArmConfig& config) const;
  ChainState computeForwardKinematics(const hardware::ArmConfig& config) const;

  // Convenience (delegates to hardware)
  size_t getNumJoints() const;
  void setJointAngle(size_t index, double angle_rad);         // radians
  void setAllJointAngles(const std::vector<double>& angles);  // radians
  std::vector<double> getAllJointAngles() const;              // radians
};

}  // namespace kinematics
}  // namespace arm
