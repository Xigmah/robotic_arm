/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-26 16:41:41
 * @modify date 2026-02-26 16:41:41
 * @desc Code for AnalyticalIK
 *       Class used to solve solveable
 *       Inverse kinematics, without need
 *       for irative joint adjustment
 */
#include "analytical_ik.h"

#include <algorithm>
#include <cmath>

using namespace std;

namespace arm {
namespace kinematics {

AnalyticIK3DOF::AnalyticIK3DOF(hardware::ArmConfig& config)
    : InverseKinematics(config) {}

double AnalyticIK3DOF::solveBaseAngle(double x, double y) const {
  return atan2(y, x);
}

optional<AnalyticIK3DOF::ShoulderElbowAngles> AnalyticIK3DOF::solve2DArm(
    double r, double z, double L_shoul, double L_elb, bool arm_bend_pos) const {
  // s -> shoulder, e -> elbow
  /*
   * if arm is positive (elbow is within triangle between
   *                      origin and r,z)
   *
   *         -1 ┌ r^2 + z^2 - Ls^2 -Le^2 ┐
   * θe = cos   │ -----------------------│
   *            └       2(Ls*Le)         ┘
   *
   *         -1 z       -1 ┌    Le*sin(θe)   ┐
   * θs = tan  --- - tan   │ --------------- │
   *            r          └ Ls + Le*cos(θe) ┘
   *
   * if arm is negative (elbow is outside triangle between
   *                      origin and r,z)
   *
   *           -1 ┌ r^2 + z^2 - Ls^2 -Le^2 ┐
   * θe = - cos   │ -----------------------│
   *              └       2(Ls*Le)         ┘
   *
   *         -1 z       -1 ┌    Le*sin(θe)   ┐
   * θs = tan  --- + tan   │ --------------- │
   *            r          └ Ls + Le*cos(θe) ┘
   */
  double shoul_angle{0.};
  double elb_angle{0.};

  /* for elb_angle, we use ArcCosine;
   * acos() domain is [-1, 1], check before moving on
   */
  double temp = (r * r + z * z - L_shoul * L_shoul - L_elb * L_elb) /
                (2 * L_shoul * L_elb);
  if (temp > 1 || temp < -1) {
    return nullopt;
  };

  if (arm_bend_pos) {
    elb_angle = acos(temp);
    shoul_angle = atan2(z, r) - atan2(L_elb * sin(elb_angle),
                                      L_shoul + L_elb * cos(elb_angle));
  } else {
    elb_angle = -acos(temp);
    shoul_angle = atan2(z, r) + atan2(L_elb * sin(elb_angle),
                                      L_shoul + L_elb * cos(elb_angle));
  }

  // If shoul_angle is outside of limits, arm not solvable
  if (!arm_config.getJoint(Joint::SHOUL)->isWithinLimits(shoul_angle) ||
      !arm_config.getJoint(Joint::ELB)->isWithinLimits(elb_angle)) {
    return nullopt;
  }

  return ShoulderElbowAngles{shoul_angle, elb_angle};
}

optional<vector<double>> AnalyticIK3DOF::solve(
    const math::Vector3D& target_position) const {
  // Enumirate joints for readability

  // Get coordinate from Vector3D
  double x{target_position.get_x()};
  double y{target_position.get_y()};
  double z{target_position.get_z()};

  // Step 1: Solve base angle
  double base_angle = solveBaseAngle(x, y);

  // Step 2: Project to 2D (distance in XY plane from shoulder)
  double r = sqrt(x * x + y * y);
  r -= arm_config.getLinkLength(Joint::BASE);
  z = -z;

  // Calculate ShoulderElbowAngle
  optional<ShoulderElbowAngles> joint_angles =
      solve2DArm(r, z, arm_config.getLinkLength(Joint::SHOUL),
                 arm_config.getLinkLength(Joint::ELB));

  // Check if we were able to solve2DArm
  if (!joint_angles.has_value()) {
    // Try opposite bend
    joint_angles = solve2DArm(r, z, arm_config.getLinkLength(Joint::SHOUL),
                              arm_config.getLinkLength(Joint::ELB), false);
  }

  // If solve2DArm still has no solution, nullopt
  if (!joint_angles.has_value()) {
    return nullopt;
  }

  return vector<double>{base_angle, joint_angles->shoulder,
                        joint_angles->elbow};
}

bool AnalyticIK3DOF::isReachable(const math::Vector3D& target) const {
  return solve(target).has_value();
}

}  // namespace kinematics
}  // namespace arm
