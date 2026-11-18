//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_SENSORS__SENSORS_HPP_
#define GAITOS_SENSORS__SENSORS_HPP_

#include <Eigen/Dense>

#include <vector>

#include "gaitos_core/SensorBase.hpp"

namespace gaitos_sensors
{

/**
 * @class ImuPassthrough
 * @brief IMU with its own orientation estimate (Unitree, Gazebo, MuJoCo): applies the
 * mounting rotation (`mounting_rpy`, IMU frame in the base frame), the gyro bias
 * (`gyro_bias`) and an optional first order low-pass on the gyro (`gyro_cutoff` [Hz], 0: off).
 */
class ImuPassthrough : public gaitos::ImuSensorBase
{
public:
  bool update(const gaitos::ImuData & raw, double dt, gaitos::ImuData & imu) override;
  void reset() override {first_ = true;}

protected:
  bool on_initialize() override;

  Eigen::Matrix3d mounting_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  double gyro_cutoff_{0.0};
  Eigen::Vector3d gyro_filtered_{Eigen::Vector3d::Zero()};
  bool first_{true};
};

/**
 * @class ComplementaryImu
 * @brief Orientation from gyro and accelerometer (Mahony filter), for IMUs without
 * orientation output or to replace it. Yaw is integrated (it drifts; external odometry
 * corrects it in the estimator). Parameters of ImuPassthrough plus `kp` (1.0) and
 * `ki` (0.01) of the accelerometer correction and `use_device_orientation` (false: always
 * estimate; true: pass through when the device gives orientation).
 */
class ComplementaryImu : public ImuPassthrough
{
public:
  bool update(const gaitos::ImuData & raw, double dt, gaitos::ImuData & imu) override;
  void reset() override;

protected:
  bool on_initialize() override;

private:
  double kp_{1.0};
  double ki_{0.01};
  bool use_device_orientation_{false};
  Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d integral_{Eigen::Vector3d::Zero()};
  bool initialized_{false};
};

/**
 * @class ForceThresholdContact
 * @brief Contact from foot force sensors (Go2 foot_force, NAO FSR, simulator contact
 * forces) with hysteresis: a foot touches when its force rises over `on_threshold` and
 * leaves when it falls under `off_threshold` (both in sensor units). If the hardware gives
 * no forces, `assume_contact` (true) sets every foot in contact.
 */
class ForceThresholdContact : public gaitos::ContactSensorBase
{
public:
  bool update(const gaitos::RobotState & state, gaitos::ContactData & contacts) override;

protected:
  bool on_initialize() override;

private:
  double on_{20.0};
  double off_{10.0};
  bool assume_contact_{true};
  std::vector<double> scale_;
};

/**
 * @class TorqueContact
 * @brief Contact without foot sensors (R1, Go2 without foot_force): the ground reaction
 * of each foot is estimated from its joint torques, F = -(J^T)^+ tau, rotated to the
 * world, and its vertical component goes through the same hysteresis as
 * ForceThresholdContact (`on_threshold`, `off_threshold` in N).
 */
class TorqueContact : public gaitos::ContactSensorBase
{
public:
  bool update(const gaitos::RobotState & state, gaitos::ContactData & contacts) override;

protected:
  bool on_initialize() override;

private:
  double on_{30.0};
  double off_{15.0};
  double filter_{0.5};   ///< Low pass on the force (0: none, close to 1: strong)
  std::vector<double> force_;
};

}  // namespace gaitos_sensors

#endif  // GAITOS_SENSORS__SENSORS_HPP_
