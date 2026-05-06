/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-19 16:48:04
 * @modify date 2026-02-19 16:48:04
 * @desc Code for Joint Factory
 *       Used to create joint of
 *       selected type.
 */

#include "joint_factory.h"

#include <stdexcept>

#include "simulated_joint.h"

namespace arm {
namespace hardware {

/* Create Joint of type
 * min_limit -> rad
 * max_limit -> rad
 * link_length -> mm
 * hardware_id -> GPIO pin, EtherCAT slave ID, etc.
 * If type unkown, runtime_error thrown
 */
std::unique_ptr<Joint> JointFactory::create(JointType type, double min_limit,
                                            double max_limit,
                                            double link_length, RotAxis axis,
                                            int hardware_id) {
  switch (type) {
    case JointType::SIMULATED:
      return std::make_unique<SimulatedJoint>(min_limit, max_limit, link_length,
                                              axis);

      // Future:
      // case JointType::ES08A_SERVO:
      //   return std::make_unique<ES08AServo>(min_limit, max_limit,
      //   link_length, hardware_id);

    default:
      throw std::runtime_error("Unknown joint type\n");
  }
}

}  // namespace hardware
}  // namespace arm
