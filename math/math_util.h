/**
 * @author razn-abuqamar
 * @email razn.abuqamar93@gmail.com
 * @create date 2026-02-16 16:35:51
 * @modify date 2026-02-16 16:35:51
 * @desc Header file for basic math utils
 */
#pragma once

#include <cmath>

inline static constexpr double def_tol = 1e-6;

inline double deg_to_rad(double deg) { return deg * (M_PI / 180); }

inline bool compare(double v1, double v2, double tol = def_tol) {
  return std::abs(v1 - v2) <= tol;
}

inline double clamp(double value, double min, double max) {
  return std::fmax(min, std::fmin(value, max));
}
