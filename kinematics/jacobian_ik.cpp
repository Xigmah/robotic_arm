/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-03-12 16:12:37
 * @modify date 2026-03-12 16:12:37
 * @desc Code for Jacobian inverse
 *       kinematics
 */

#include "jacobian_ik.h"

#include "kinematic_chain.h"

namespace arm {
namespace kinematics {

JacobianIK::JacobianIK(hardware::ArmConfig& config)
    : InverseKinematics(config) {}

/*
 * Yes — looking at your computeForwardKinematics:
 *
 * Everything else looks good — the loop, the cross product, pushing columns
 * into the vector. Now you need to think about the next step: you have J as a
 * std::vector<Vector3D> (3×N matrix), and you have the error vector. How do you
 * compute Δθ from those two things?
 */
std::optional<std::vector<double>> JacobianIK::solve(
    const math::Vector3D& target_position) const {
  std::vector<double> joint_angles{};
  hardware::ArmConfig temp{arm_config};
  KinematicChain comp{temp};

  // ChainState used for Jacobian comp
  ChainState joint_state{comp.computeForwardKinematics(temp)};

  math::Vector3D end_eff_pos{joint_state.end_eff_vec};

  math::Vector3D error = target_position - end_eff_pos;  // vector
  double error_magnitude = error.magnitude();  // scalar for convergence check

  std::vector<math::Vector3D> jacobian{};
  for (auto frame : joint_state.joint_frames) {
    // J_column_i = axis_i × (end_effector_pos - joint_i_pos)
    math::Vector3D jacobian_column{
        frame.axis.cross(end_eff_pos - frame.joint_pos)};

    jacobian.push_back(jacobian_column);
  }

  std::vector<math::Vector3D> delta_theta{};
  for (auto column : jacobian) {
    delta_theta.push_back(column.dot(error));
  }
}

}  // namespace kinematics
}  // namespace arm
