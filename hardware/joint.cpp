/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-19 16:51:31
 * @modify date 2026-02-19 16:51:31
 * @desc Code for JointClass
 *       Interface class to abstract
 *       joints and hardware necessities
 */

#include "joint.h"

namespace arm {
namespace hardware {

/* Construct Joint
 * min -> rad
 * max -> rad
 * length -> mm
 * Default angle is in middle of range
 */
Joint::Joint(double min, double max, double length)
    : min_limit{min}, max_limit{max}, link_length{length} {
  current_angle = (max_limit + min_limit) / 2;  // Middle of range
}

// Check if angle within limits
bool Joint::isWithinLimits(double angle_rad) const {
  return angle_rad <= max_limit && angle_rad >= min_limit;
}

}  // namespace hardware
}  // namespace arm
