/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-03-12 16:33:10
 * @modify date 2026-03-12 16:33:10
 * @desc Utility file for code in kinematics
 */
#pragma once

#include <array>

namespace arm {
namespace kinematics {

enum Joint { BASE = 0, SHOUL = 1, ELB = 2, WRIST = 3 };

struct ShoulderElbowAngles {
  double shoulder;  // θ1
  double elbow;     // θ2
};

struct JointFrame {
  double x, y, z;
  std::array<double, 3> axis;
};

struct ChainState {
  math::Vector3D end_eff_vec;
  std::vector<JointFrame> joint_frames;
};

}  // namespace kinematics
}  // namespace arm
