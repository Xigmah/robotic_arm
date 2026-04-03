/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-19 16:52:21
 * @modify date 2026-02-19 16:52:21
 * @desc Code for SimulatedJoint
 *       Mock joint used for testing kinematics
 */

#include "simulated_joint.h"

#include <algorithm>
#include <iostream>

namespace arm {
namespace hardware {

/* Construct simulated joint
 * min -> rad
 * max -> rad
 * length -> mm
 */
SimulatedJoint::SimulatedJoint(double min, double max, double length,
                               RotAxis axis)
    : Joint(min, max, length, axis) {}

/* Set Target to travel to
 * angle -> rad
 * If angle is greated than max, user is warned
 * and command is clamped
 */
void SimulatedJoint::setTargetAngle(double angle_rad) {
  if (Joint::isWithinLimits(angle_rad)) {
    current_angle = angle_rad;
  } else {
    std::cerr << "angle out of bounds, clamping";
    std::cout << "\nangle out of bounds, clamping" << std::endl;
    current_angle = std::clamp(angle_rad, min_limit, max_limit);
  }
}

}  // namespace hardware
}  // namespace arm
