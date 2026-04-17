/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-19 16:52:56
 * @modify date 2026-02-19 16:52:56
 * @desc Header for SimulatedJoint
 *       Mock joint used for testing kinematics
 */
#pragma once

#include "joint.h"

namespace arm {
namespace hardware {

class SimulatedJoint : public Joint {
 public:
  SimulatedJoint(double min, double max, double length, RotAxis axis);
  ~SimulatedJoint() = default;

  void setTargetAngle(double angle_rad) override;
  double getCurrentAngle() const override { return current_angle; }
  std::unique_ptr<Joint> clone() const override;
};

}  // namespace hardware
}  // namespace arm
