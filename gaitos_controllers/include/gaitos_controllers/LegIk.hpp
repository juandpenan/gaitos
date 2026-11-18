//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CONTROLLERS__LEGIK_HPP_
#define GAITOS_CONTROLLERS__LEGIK_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>

#include "gaitos_common/Types.hpp"
#include "gaitos_core/Gains.hpp"
#include "gaitos_model/RobotModel.hpp"

namespace gaitos_controllers
{

/**
 * @class LegIk
 * @brief Foot placement helper shared by the controllers: nominal stance from an SRDF
 * pose, inverse kinematics of every leg (warm started), joint velocity feedforward and
 * gravity compensation of the stance legs.
 */
class LegIk
{
public:
  /**
   * @param model robot
   * @param stance_pose SRDF pose of the nominal stance
   */
  void configure(gaitos::RobotModelPtr model, const std::string & stance_pose);

  /// Joint vector of the stance pose
  const std::vector<double> & stance_q() const {return stance_q_;}

  /// Feet of the stance pose, base frame
  const std::vector<Eigen::Isometry3d> & nominal_feet() const {return nominal_;}

  /// Height of the base over the feet in the stance pose [m]
  double nominal_height() const {return height_;}

  /**
   * @brief Solve the joints of every leg for the given foot poses (base frame)
   * @param feet desired foot poses; only the position is used unless the leg has
   * orientation_ik
   * @param[in,out] q model joint vector, warm start and result
   * @return max position error [m]
   */
  double solve(const std::vector<Eigen::Isometry3d> & feet, std::vector<double> & q) const;

  /**
   * @brief Joint velocities of one leg for a foot velocity (base frame)
   */
  void leg_velocity(
    std::size_t leg, const std::vector<double> & q, const Eigen::Vector3d & foot_velocity,
    std::vector<double> & dq) const;

  /**
   * @brief Feedforward torques so that the legs in `stance` carry the weight
   * @param orientation base orientation (world)
   * @param stance legs in contact
   * @param fraction part of the weight to compensate (0..1)
   */
  void gravity_compensation(
    const std::vector<double> & q, const Eigen::Quaterniond & orientation,
    const std::vector<bool> & stance, double fraction, std::vector<double> & tau) const;

  /**
   * @brief Foot poses with the body moved by (offset, rpy) over the nominal stance, the feet
   * staying where they are
   */
  std::vector<Eigen::Isometry3d> body_pose_feet(
    const std::vector<Eigen::Isometry3d> & feet, const Eigen::Vector3d & offset,
    const Eigen::Vector3d & rpy) const;

private:
  gaitos::RobotModelPtr model_;
  std::vector<double> stance_q_;
  std::vector<Eigen::Isometry3d> nominal_;
  double height_{0.0};
  bool coupled_{false};   ///< legs share joints (e.g. the HipYawPitch motor of the NAO)
};

/**
 * @brief Linear interpolation of two joint vectors
 */
void blend(const std::vector<double> & from, const std::vector<double> & to, double alpha,
  std::vector<double> & out);

}  // namespace gaitos_controllers

#endif  // GAITOS_CONTROLLERS__LEGIK_HPP_
