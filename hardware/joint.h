/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-19 16:50:20
 * @modify date 2026-02-19 16:50:20
 * @desc Header for JointClass
 *       Interface class to abstract
 *       joints and hardware necessities
 */
#pragma once
#include <cmath>

namespace arm {
namespace hardware {

class Joint {
 private:
  static constexpr double def_min_limit = 0.0;     // rad
  static constexpr double def_max_limit = M_2_PI;  // rad
  static constexpr double def_link_length = 10.0;  // mm

 protected:
  double current_angle; // rad
  double min_limit;     // rad
  double max_limit;     // rad
  double link_length;   // mm

 public:
  Joint(double min = def_min_limit, double max = def_max_limit,
        double length = def_link_length);
  virtual ~Joint() = default;

  // Pure virtual - subclasses must implement
  virtual void setTargetAngle(double angle_rad) = 0;
  virtual double getCurrentAngle() const = 0;

  // Common functionality
  bool isWithinLimits(double angle_rad) const;

  // Getters
  double getMinLimit() const { return min_limit; }
  double getMaxLimit() const { return max_limit; }
  double getLinkLength() const { return link_length; }
};

}  // namespace hardware
}  // namespace arm
