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
  double step_size{0.00001};
  double tolerance{1.0};
  size_t max_iteration{10000};

 protected:
  size_t iterations{0};

 public:
  explicit JacobianIK(hardware::ArmConfig& config);

  // Returns joint angles if solution exists, nullopt if unreachable
  std::optional<std::vector<double>> solve(
      const math::Vector3D& target_position) const override;

  // Getters
  double get_tol() const { return tolerance; };
  double get_step() const { return step_size; };
  size_t get_max_itr() const { return max_iteration; };

  // Setters
  void set_tol(const double tol) { tolerance = tol; };
  void set_step(const double step) { step_size = step; };
  void set_max_itr(const size_t max) { max_iteration = max; };

  // Helper - check if target is within reach
  bool isReachable(const math::Vector3D& target) const override;
};

}  // namespace kinematics
}  // namespace arm
