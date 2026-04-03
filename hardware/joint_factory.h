/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-19 16:49:14
 * @modify date 2026-02-19 16:49:14
 * @desc Header for Joint Factory
 *       Used to create joint of
 *       selected type.
 */
#pragma once

#include <memory>

#include "joint.h"

namespace arm {
namespace hardware {

enum class JointType {
  SIMULATED,
  ES08A_SERVO,  // Future
  ETHERCAT      // Future
};

class JointFactory {
 public:
  static std::unique_ptr<Joint> create(
      JointType type, double min_limit, double max_limit, double link_length,
      RotAxis axis,
      int hardware_id = 0  // GPIO pin, EtherCAT slave ID, etc.
  );
};

}  // namespace hardware
}  // namespace arm
