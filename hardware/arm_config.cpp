/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-19 16:47:27
 * @modify date 2026-02-19 16:47:27
 * @desc Code for Arm config
 *       Controls arm configuration
 */

#include "arm_config.h"

#include <stdexcept>

namespace arm {
namespace hardware {

// Deep copy for ArmConfig
ArmConfig::ArmConfig(const ArmConfig& source) {
  for (const auto& joint_ptr : source.joints) {
    joints.push_back(joint_ptr->clone());
  }
}

// Add Joint
void ArmConfig::addJoint(std::unique_ptr<Joint> joint) {
  joints.push_back(std::move(joint));
}

// Get Joint
Joint* ArmConfig::getJoint(size_t index) const {
  if (index >= joints.size()) {
    throw std::out_of_range("Selected joint index out of range");
  }
  return joints[index].get();
}

/* Get Joint link_length -> mm
 * if index out of range, throws
 */
double ArmConfig::getLinkLength(size_t index) const {
  if (index >= joints.size()) {
    throw std::out_of_range("Selected joint index out of range");
  }
  return joints[index]->getLinkLength();
}

/* Get Joint angle -> rad
 * if index out of range, throws
 */
double ArmConfig::getJointAngle(size_t index) const {
  if (index >= joints.size()) {
    throw std::out_of_range("Selected joint index out of range");
  }
  return joints[index]->getCurrentAngle();
}

/* Sets Joint angle -> rad
 * if index out of range, throws
 */
void ArmConfig::setJointAngle(size_t index, double angle_rad) {
  if (index >= joints.size()) {
    throw std::out_of_range("Selected joint index out of range");
  }
  joints[index]->setTargetAngle(angle_rad);
}

/* Sets all Joint angles -> rad
 * angles vector mismatches size, throws
 */
void ArmConfig::setAllJointAngles(const std::vector<double>& angles) {
  if (angles.size() != joints.size()) {
    throw std::runtime_error("Unmatched number of angles to number of joints");
  }
  for (size_t i{0}; i < joints.size(); i++) {
    ArmConfig::setJointAngle(i, angles[i]);
  }
}

// Gets all Joint angles -> rad
std::vector<double> ArmConfig::getAllJointAngles() const {
  std::vector<double> temp{};
  for (const auto& joint : joints) {
    temp.push_back(joint->getCurrentAngle());
  }
  return temp;
}

}  // namespace hardware
}  // namespace arm
