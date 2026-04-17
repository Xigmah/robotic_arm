/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-25 21:37:22
 * @modify date 2026-02-25 21:37:22
 * @desc header file for InverseKinematics
 *       Interface class
 */
#pragma once

#include <optional>

#include "../hardware/arm_config.h"
#include "kinematics_util.h"

namespace arm {
namespace kinematics {

class InverseKinematics {
 protected:
  hardware::ArmConfig& arm_config;

  // Helpers for subclasses
  double getTotalReach() const;  // Sum of all link lengths

 public:
  explicit InverseKinematics(hardware::ArmConfig& config);
  virtual ~InverseKinematics() = default;

  // Returns joint angles if solution exists, nullopt if unreachable
  virtual std::optional<std::vector<double>> solve(
      const math::Vector3D& target_position) const = 0;

  // Helper - check if target is within reach
  virtual bool isReachable(const math::Vector3D& target) const = 0;
};

}  // namespace kinematics
}  // namespace arm
