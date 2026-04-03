/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-19 16:37:25
 * @modify date 2026-02-19 16:37:25
 * @desc Header for Arm config
 *       Controls arm configuration
 */
#pragma once

#include <memory>
#include <vector>

#include "joint.h"

namespace arm {
namespace hardware {

class ArmConfig {
 private:
  std::vector<std::unique_ptr<Joint>> joints;

 public:
  ArmConfig() = default;
  ~ArmConfig() = default;

  // Build configuration
  void addJoint(std::unique_ptr<Joint> joint);

  // Queries
  size_t getNumJoints() const { return joints.size(); }
  RotAxis getJointRotAxis(size_t index) const {
    return joints[index]->getRotAxis();
  };
  Joint* getJoint(size_t index) const;

  double getLinkLength(size_t index) const;
  double getJointAngle(size_t index) const;

  // Joint control
  void setJointAngle(size_t index, double angle_rad);
  void setAllJointAngles(const std::vector<double>& angles);
  std::vector<double> getAllJointAngles() const;
};

}  // namespace hardware
}  // namespace arm
