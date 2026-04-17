/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-03-12 16:33:10
 * @modify date 2026-03-12 16:33:10
 * @desc Utility file for code in kinematics
 */
#pragma once

#include <vector>
#include "../math/vector3d.h"

namespace arm {
namespace kinematics {

enum Joint { BASE = 0, SHOUL = 1, ELB = 2, WRIST = 3 };

struct ShoulderElbowAngles {
  double shoulder;  // θ1
  double elbow;     // θ2
};

struct JointFrame {
  math::Vector3D joint_pos, axis;
};

struct ChainState {
  math::Vector3D end_eff_vec;
  std::vector<JointFrame> joint_frames;
};

}  // namespace kinematics
}  // namespace arm
