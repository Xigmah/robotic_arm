/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-03-12 16:12:37
 * @modify date 2026-03-12 16:12:37
 * @desc Code for Jacobian inverse
 *       kinematics
 */

#include "jacobian_ik.h"

namespace arm {
namespace kinematics {

JacobianIK::JacobianIK(hardware::ArmConfig& config)
    : InverseKinematics(config) {}

std::optional<std::vector<double>> JacobianIK::solve(
    const math::Vector3D& target_position) const {
  std::vector<double> joint_angles{arm_config.getAllJointAngles()};
  hardware::ArmConfig temp{arm_config};
  KinematicChain comp{temp};

  math::Vector3D current_pos{comp.computFKTranslation(temp)};

  math::Vector3D error = target_position - current_pos;  // vector
  double error_magnitude = error.magnitude();  // scalar for convergence check
}

}  // namespace kinematics
}  // namespace arm
