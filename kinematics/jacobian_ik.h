/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-03-12 16:12:33
 * @modify date 2026-03-12 16:12:33
 * @desc Header for Jacobian inverse
 *       kinematics
 */
#pragma once

#include "inverse_kinematics.h"

namespace arm {
namespace kinematics {

class JacobianIK : public InverseKinematics {
 private:
  static constexpr double step_size{0.01};
  static constexpr size_t max_iteration{100};
 protected:
 public:
  explicit JacobianIK(hardware::ArmConfig& config);

  // Returns joint angles if solution exists, nullopt if unreachable
  std::optional<std::vector<double>> solve(
      const math::Vector3D& target_position) const override;

  // Helper - check if target is within reach
  bool isReachable(const math::Vector3D& target) const override;
};

}  // namespace kinematics
}  // namespace arm
