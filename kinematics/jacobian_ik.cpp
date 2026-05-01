/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-03-12 16:12:37
 * @modify date 2026-03-12 16:12:37
 * @desc Code for Jacobian inverse
 *       kinematics
 */

#include "jacobian_ik.h"

// #include <iostream>
#include <algorithm>
#include <stdexcept>

#include "kinematic_chain.h"

namespace arm {
namespace kinematics {

JacobianIK::JacobianIK(hardware::ArmConfig& config)
    : InverseKinematics(config) {}

std::optional<std::vector<double>> JacobianIK::solve(
    const math::Vector3D& target_position) const {
  // Initializing angles for return
  std::vector<double> joint_angles{arm_config.getAllJointAngles()};

  // Initializing a copy of arm_config for use when looping
  hardware::ArmConfig temp{arm_config};
  KinematicChain comp{temp};

  // ChainState used for Jacobian comp
  ChainState joint_state{comp.computeForwardKinematics(temp)};

  // Check joint_state.joint_frames vector is the same size as joint_angles
  if (joint_state.joint_frames.size() != arm_config.getNumJoints()) {
    throw std::out_of_range("Joint frames do not match number of joints\n");
  }

  math::Vector3D end_eff_pos{joint_state.end_eff_vec};
  math::Vector3D error = target_position - end_eff_pos;  // vector
  double error_magnitude = error.magnitude();  // scalar for convergence check

  // Set iteration counter, nullopt if max iterations reached
  size_t iteration{0};
  while (error_magnitude >= tolerance) {
    // nullopt if max iterations reached
    if (iteration >= max_iteration) {
      printf("Unable to reach target before reaching max iterations\n");
      return std::nullopt;
    }

    /* Instantiate Jacobian,
     * Set jacobian back to empty in new while loop
     */
    std::vector<math::Vector3D> jacobian{};
    for (auto frame : joint_state.joint_frames) {
      // J_column_i = axis_i × (end_effector_pos - joint_i_pos)
      math::Vector3D jacobian_column{
          frame.axis.cross(end_eff_pos - frame.joint_pos)};

      jacobian.push_back(jacobian_column);
    }

    /* Instantiate Δθ,
     * Set Δθ back to empty in new while loop
     */
    std::vector<double> delta_theta{};
    for (auto column : jacobian) {
      delta_theta.push_back(column.dot(error));
    }

    /* Set delta_theta to joints and compute
     * new forwardkinematics, then compute new ChainState.
     * For setting new angles, use step_size to not overshoot
     */
    joint_angles = temp.getAllJointAngles();
    for (size_t i = 0; i < joint_angles.size(); i++) {
      joint_angles[i] += step_size * delta_theta[i];

      // Clamp Angle
      auto joint = arm_config.getJoint(i);
      joint_angles[i] = std::clamp(joint_angles[i], joint->getMinLimit(),
                                   joint->getMaxLimit());
    }
    // Perturb initial angles to avoid singularity
    if (iteration == 0) {
      joint_angles[1] = 0.1;  // small shoulder angle
    }
    temp.setAllJointAngles(joint_angles);
    joint_state = comp.computeForwardKinematics(temp);
    end_eff_pos = joint_state.end_eff_vec;

    // Check joint_state.joint_frames vector size is same as number of joints
    if (joint_state.joint_frames.size() != arm_config.getNumJoints()) {
      throw std::out_of_range("Joint frames do not match number of joints\n");
    }

    // Set new error
    error = target_position - joint_state.end_eff_vec;
    error_magnitude = error.magnitude();
    // printf("error: %0.2f\n", error_magnitude);
    iteration++;
  }

  /* If we've returned from the loop without nullopt,
   * then we should have have an error_magnitude less than
   * or equal to our tolerance, and the currently stored
   * joint_angles should be our usable angles
   */
  return joint_angles;
}

bool JacobianIK::isReachable(const math::Vector3D& target) const {
  return solve(target).has_value();
}

}  // namespace kinematics
}  // namespace arm
