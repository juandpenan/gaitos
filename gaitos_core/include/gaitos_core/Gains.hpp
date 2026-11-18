//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CORE__GAINS_HPP_
#define GAITOS_CORE__GAINS_HPP_

#include <vector>

#include "gaitos_common/Params.hpp"
#include "gaitos_model/RobotModel.hpp"

namespace gaitos
{

/**
 * @struct JointGains
 * @brief Stiffness and damping of every joint.
 */
struct JointGains
{
  std::vector<double> kp;
  std::vector<double> kd;
};

/**
 * @brief Read per-joint gains. The block maps a joint name, a group ("all", "legs",
 * "upper_body" or an SRDF group) or "default" to {kp, kd}; "default" is applied first,
 * then the groups, then the single joints:
 *
 * @code{.yaml}
 * gains:
 *   default: {kp: 40.0, kd: 1.0}
 *   legs: {kp: 60.0, kd: 2.0}
 *   FR_calf_joint: {kp: 80.0}
 * @endcode
 */
JointGains read_gains(const Params & gains, const RobotModel & model, double kp = 0.0, double kd = 0.0);

/**
 * @brief Joints from a list of names and groups (the `joints` parameter of the plugins)
 */
std::vector<std::size_t> read_joint_selection(
  const Params & params, const std::string & key, const RobotModel & model,
  const std::vector<std::string> & default_value = {"all"});

}  // namespace gaitos

#endif  // GAITOS_CORE__GAINS_HPP_
