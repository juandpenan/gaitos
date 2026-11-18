//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

// Helpers shared by the dynamic controllers (internal header)

#ifndef GAITOS_DYNAMICS__COMMON_HPP_
#define GAITOS_DYNAMICS__COMMON_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

#include "gaitos_common/Math.hpp"
#include "gaitos_common/Types.hpp"

namespace gaitos_dynamics
{
namespace detail
{
inline Eigen::Matrix2d rot2(double a)
{
  Eigen::Matrix2d r;
  r << std::cos(a), -std::sin(a), std::sin(a), std::cos(a);
  return r;
}

inline double mean_angle(double a, double b)
{
  return gaitos::wrap_angle(a + 0.5 * gaitos::wrap_angle(b - a));
}

inline double cycloid(double s)
{
  s = std::clamp(s, 0.0, 1.0);
  return s - std::sin(2.0 * M_PI * s) / (2.0 * M_PI);
}

inline Eigen::Vector3d limited_command(
  const gaitos::ControlReference & ref, const Eigen::Vector3d & max_v)
{
  return {std::clamp(ref.linear_velocity.x(), -max_v.x(), max_v.x()),
    std::clamp(ref.linear_velocity.y(), -max_v.y(), max_v.y()),
    std::clamp(ref.angular_velocity.z(), -max_v.z(), max_v.z())};
}
}  // namespace detail
}  // namespace gaitos_dynamics

#endif  // GAITOS_DYNAMICS__COMMON_HPP_
