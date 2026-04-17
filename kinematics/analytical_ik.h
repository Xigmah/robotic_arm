/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-25 21:38:12
 * @modify date 2026-02-25 21:38:12
 * @desc Header file for AnalyticalIK
 *       Class used to solve solveable
 *       Inverse kinematics, without need
 *       for irative joint adjustment
 */
#pragma once

#include "inverse_kinematics.h"

namespace arm {
namespace kinematics {

// Analytical IK solver for 3-DOF RRR arm (Z-Y-Y configuration)
class AnalyticIK3DOF : public InverseKinematics {
 private:
  std::optional<ShoulderElbowAngles> solve2DArm(
      double r,                 // horizontal distance
      double z,                 // vertical distance
      double L_shoul,           // link 2 length
      double L_elb,             // link 3 length
      bool arm_bend_pos = true  // Arm Bend, default positive
  ) const;

  double solveBaseAngle(double x, double y) const;

 public:
  explicit AnalyticIK3DOF(hardware::ArmConfig& config);

  std::optional<std::vector<double>> solve(
      const math::Vector3D& target_position) const override;

  // Helper - check if target is within reach
  bool isReachable(const math::Vector3D& target) const override;
};

}  // namespace kinematics
}  // namespace arm
