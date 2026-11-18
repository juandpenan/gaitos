//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CORE__JOINTACTUATORBASE_HPP_
#define GAITOS_CORE__JOINTACTUATORBASE_HPP_

#include <vector>

#include "gaitos_common/Types.hpp"
#include "gaitos_core/PluginBase.hpp"

namespace gaitos
{

/**
 * @class JointActuatorBase
 * @brief Model of the motors of a group of joints: turns the controller command into what
 * the motor drivers of those joints accept (PD in the driver, torque only, position only,
 * transmissions, limits...).
 *
 * The `joints` parameter selects the joints (a list of joint names or groups: "all",
 * "legs", "upper_body" or an SRDF group). Several actuator plugins may be configured, each
 * one for different joints.
 */
class JointActuatorBase : public PluginBase
{
public:
  /**
   * @brief Joints handled by this instance (model indices)
   */
  const std::vector<std::size_t> & joints() const {return joints_;}
  void set_joints(std::vector<std::size_t> joints) {joints_ = std::move(joints);}

  /**
   * @brief Compute the motor command of its joints
   * @param measured joint state
   * @param command controller output
   * @param[out] output command to the hardware (only its joints are written)
   */
  virtual void update(
    const JointState & measured, const JointCommand & command, JointCommand & output) = 0;

private:
  std::vector<std::size_t> joints_;
};

}  // namespace gaitos

#endif  // GAITOS_CORE__JOINTACTUATORBASE_HPP_
