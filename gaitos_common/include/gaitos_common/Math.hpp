//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_COMMON__MATH_HPP_
#define GAITOS_COMMON__MATH_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace gaitos
{

constexpr double kGravity = 9.81;

/**
 * @brief Rotation from roll, pitch, yaw (Z-Y-X intrinsic: R = Rz(yaw) Ry(pitch) Rx(roll))
 */
inline Eigen::Matrix3d rpy_to_matrix(const Eigen::Vector3d & rpy)
{
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

inline Eigen::Quaterniond rpy_to_quaternion(const Eigen::Vector3d & rpy)
{
  return Eigen::Quaterniond(rpy_to_matrix(rpy));
}

/**
 * @brief Roll, pitch, yaw of a rotation (inverse of rpy_to_matrix)
 */
inline Eigen::Vector3d matrix_to_rpy(const Eigen::Matrix3d & r)
{
  const double pitch = std::asin(std::clamp(-r(2, 0), -1.0, 1.0));
  const double roll = std::atan2(r(2, 1), r(2, 2));
  const double yaw = std::atan2(r(1, 0), r(0, 0));
  return {roll, pitch, yaw};
}

inline Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond & q)
{
  return matrix_to_rpy(q.normalized().toRotationMatrix());
}

inline double yaw_of(const Eigen::Quaterniond & q)
{
  const Eigen::Quaterniond n = q.normalized();
  return std::atan2(
    2.0 * (n.w() * n.z() + n.x() * n.y()),
    1.0 - 2.0 * (n.y() * n.y() + n.z() * n.z()));
}

/**
 * @brief Wrap an angle to (-pi, pi]
 */
inline double wrap_angle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

/**
 * @brief Tilt of the base: angle between its z axis and the world z axis [rad]
 */
inline double tilt_angle(const Eigen::Quaterniond & q)
{
  const Eigen::Vector3d z = q.normalized() * Eigen::Vector3d::UnitZ();
  return std::acos(std::clamp(z.z(), -1.0, 1.0));
}

/**
 * @brief Skew symmetric matrix of a vector
 */
inline Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

/**
 * @brief Rotation error as a rotation vector (axis * angle) that takes `current` to `desired`,
 * in the frame where both are expressed
 */
inline Eigen::Vector3d rotation_error(const Eigen::Matrix3d & desired, const Eigen::Matrix3d & current)
{
  const Eigen::AngleAxisd aa(desired * current.transpose());
  return aa.axis() * aa.angle();
}

/**
 * @brief Smooth 0 -> 1 interpolation (cubic, zero velocity at both ends)
 */
inline double smoothstep(double s)
{
  s = std::clamp(s, 0.0, 1.0);
  return s * s * (3.0 - 2.0 * s);
}

/**
 * @brief Clamp the norm of a vector
 */
inline Eigen::Vector3d clamp_norm(const Eigen::Vector3d & v, double max_norm)
{
  const double n = v.norm();
  return (n > max_norm && n > 0.0) ? Eigen::Vector3d(v * (max_norm / n)) : v;
}

/**
 * @brief Move a value towards a target with a maximum rate
 */
inline double rate_limit(double current, double target, double max_rate, double dt)
{
  const double step = max_rate * dt;
  return current + std::clamp(target - current, -step, step);
}

}  // namespace gaitos

#endif  // GAITOS_COMMON__MATH_HPP_
