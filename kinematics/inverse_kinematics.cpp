/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-26 16:41:41
 * @modify date 2026-02-26 16:41:41
 * @desc Code for InverseKinematics
 *       Interface class
 */
#include "inverse_kinematics.h"

namespace arm {
namespace kinematics {

InverseKinematics::InverseKinematics(hardware::ArmConfig& config)
    : arm_config{config} {}

double InverseKinematics::getTotalReach() const {
  size_t num_joints{arm_config.getNumJoints()};
  double result{0.0};
  for (size_t idx{0}; idx < num_joints; idx++) {
    result += arm_config.getLinkLength(idx);
  }
  return result;
}

}  // namespace kinematics
}  // namespace arm
