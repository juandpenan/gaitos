//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_ESTIMATORS__ESTIMATORS_HPP_
#define GAITOS_ESTIMATORS__ESTIMATORS_HPP_

#include <Eigen/Dense>

#include <map>
#include <string>
#include <vector>

#include "gaitos_core/StateEstimatorBase.hpp"

namespace gaitos_estimators
{

/**
 * @struct LegMeasurement
 * @brief Kinematics of a foot relative to the base.
 */
struct LegMeasurement
{
  Eigen::Vector3d position;   ///< Foot in the base frame
  Eigen::Vector3d velocity;   ///< Foot velocity relative to the base, base frame (J dq)
};

/**
 * @brief Foot positions and velocities from the joint state
 */
void leg_kinematics(
  const gaitos::RobotModel & model, const gaitos::JointState & joints,
  std::vector<LegMeasurement> & legs);

/**
 * @class ExternalSource
 * @brief Configuration and alignment of an external odometry source (motion capture,
 * visual or lidar odometry, simulator ground truth...).
 *
 * @code{.yaml}
 * external:
 *   <source name>:            # key of state.external_odometry
 *     position: true          # fuse the position
 *     velocity: false         # fuse the linear velocity (body frame in the message)
 *     yaw: false              # correct the yaw drift of the IMU
 *     position_noise: 0.02    # [m] std, when the message has no covariance
 *     velocity_noise: 0.1     # [m/s] std
 *     yaw_gain: 0.02          # complementary filter gain per message
 *     align: true             # the source frame is not the estimator frame: align them on
 *                             # the first message (yaw and translation)
 *     timeout: 0.5            # [s] ignore older messages
 *     gate: 0.0               # Mahalanobis (chi2) gate, 0: no outlier rejection
 * @endcode
 */
struct ExternalSource
{
  std::string name;
  bool position{true};
  bool velocity{false};
  bool yaw{false};
  double position_noise{0.02};
  double velocity_noise{0.1};
  double yaw_gain{0.02};
  bool align{true};
  double timeout{0.5};
  double gate{0.0};

  // Runtime
  uint64_t last_seq{0};
  bool aligned{false};
  Eigen::Matrix3d align_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d align_translation{Eigen::Vector3d::Zero()};
  std::size_t accepted{0};
  std::size_t rejected{0};

  static ExternalSource from_params(const std::string & name, const gaitos::Params & p);

  /**
   * @brief Position and yaw of a message in the estimator frame
   */
  Eigen::Vector3d to_world(const Eigen::Vector3d & p) const {return align_rotation * p + align_translation;}
  double yaw_to_world(double yaw) const;
};

/**
 * @class KalmanEstimator
 * @brief Linear Kalman filter of the base and the feet (the MIT Cheetah estimator) plus
 * external odometries.
 *
 * State: base position and velocity (world) and the position of each foot (world).
 * Prediction with the IMU acceleration rotated by the IMU orientation; corrections with
 * the foot positions given by the leg kinematics, the zero velocity of the feet in
 * contact, the height of the feet in contact (`flat_ground`) and the external odometry
 * sources (`external`). Feet in the air get a large process noise, so their state follows
 * the kinematics. The orientation is the IMU one, with the yaw drift corrected by the
 * sources with `yaw: true`.
 *
 * Parameters: process_noise.{position, velocity, foot, foot_swing},
 * measurement_noise.{foot_position, foot_velocity, foot_height}, flat_ground (true),
 * ground_height (0), initialize_from (source name, optional), external (see ExternalSource).
 */
class KalmanEstimator : public gaitos::StateEstimatorBase
{
public:
  bool update(const gaitos::RobotState & state, gaitos::BaseState & estimate) override;
  void reset() override {initialized_ = false;}

  const std::map<std::string, ExternalSource> & sources() const {return sources_;}

protected:
  bool on_initialize() override;

private:
  void initialize(const gaitos::RobotState & state, const Eigen::Matrix3d & r);
  void predict(const Eigen::Vector3d & acc_world, double dt, const gaitos::ContactData & contacts);
  bool correct(const Eigen::VectorXd & y, const Eigen::MatrixXd & h, const Eigen::VectorXd & r_diag, double gate);
  void fuse_external(const gaitos::RobotState & state, Eigen::Matrix3d & r);

  std::size_t legs_{0};
  Eigen::VectorXd x_;
  Eigen::MatrixXd p_;
  double q_position_{0.002};
  double q_velocity_{0.05};
  double q_foot_{0.002};
  double q_foot_swing_{100.0};
  double r_foot_position_{0.005};
  double r_foot_velocity_{0.05};
  double r_foot_height_{0.01};
  double r_swing_{1e4};
  bool flat_ground_{true};
  double ground_height_{0.0};
  std::string initialize_from_;
  double yaw_offset_{0.0};
  bool initialized_{false};
  std::map<std::string, ExternalSource> sources_;
  std::vector<LegMeasurement> leg_meas_;
};

/**
 * @class LegOdometryEstimator
 * @brief Dead reckoning without a filter: orientation from the IMU, velocity as the mean
 * of the velocities given by the feet in contact, height from those feet on flat ground
 * (`flat_ground`, true), position integrated. Optional `velocity_filter` (0.5, low pass).
 */
class LegOdometryEstimator : public gaitos::StateEstimatorBase
{
public:
  bool update(const gaitos::RobotState & state, gaitos::BaseState & estimate) override;
  void reset() override {initialized_ = false;}

protected:
  bool on_initialize() override;

private:
  bool flat_ground_{true};
  double filter_{0.5};
  bool initialized_{false};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
  std::vector<LegMeasurement> leg_meas_;
};

/**
 * @class ExternalOdometryEstimator
 * @brief The base state is an external odometry source (`source`, default "ground_truth"),
 * e.g. the simulator ground truth or a motion capture system. With `use_imu_orientation`
 * the orientation and angular velocity come from the IMU. Invalid after `timeout` s
 * without messages.
 */
class ExternalOdometryEstimator : public gaitos::StateEstimatorBase
{
public:
  bool update(const gaitos::RobotState & state, gaitos::BaseState & estimate) override;

protected:
  bool on_initialize() override;

private:
  std::string source_{"ground_truth"};
  bool use_imu_orientation_{false};
  double timeout_{0.5};
  uint64_t last_seq_{0};
};

}  // namespace gaitos_estimators

#endif  // GAITOS_ESTIMATORS__ESTIMATORS_HPP_
