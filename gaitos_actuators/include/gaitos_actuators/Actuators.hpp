//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_ACTUATORS__ACTUATORS_HPP_
#define GAITOS_ACTUATORS__ACTUATORS_HPP_

#include <vector>

#include "gaitos_core/JointActuatorBase.hpp"

namespace gaitos_actuators
{

/**
 * @class LimitsBase
 * @brief Common limits of the actuator plugins: position targets within the joint limits
 * (minus `position_margin`), torques within `torque_scale` times the URDF effort limit
 * (or `max_torque`), and gains up to `max_kp` / `max_kd`.
 */
class LimitsBase : public gaitos::JointActuatorBase
{
protected:
  bool on_initialize() override;
  double clamp_position(std::size_t i, double q) const;
  double clamp_torque(std::size_t i, double tau) const;

  std::vector<double> torque_limit_;   ///< Per model joint, <= 0: none
  double position_margin_{0.0};
  double max_kp_{1e9};
  double max_kd_{1e9};
};

/**
 * @class PdPassthrough
 * @brief Motors with the PD loop in the driver (Unitree motors, simulators with joint
 * PD): the command goes through as (q, dq, kp, kd, tau), only limited.
 */
class PdPassthrough : public LimitsBase
{
public:
  void update(
    const gaitos::JointState & measured, const gaitos::JointCommand & command,
    gaitos::JointCommand & output) override;
};

/**
 * @class TorquePd
 * @brief Motors that only accept torque: the PD is closed here, at the rate of the gaitos
 * loop, and the output has kp = kd = 0.
 */
class TorquePd : public LimitsBase
{
public:
  void update(
    const gaitos::JointState & measured, const gaitos::JointCommand & command,
    gaitos::JointCommand & output) override;
};

/**
 * @class PositionServo
 * @brief Position controlled servos (e.g. NAO, hobby servos): the output is the position
 * target and a stiffness. Joints commanded with kp = 0 are made passive (stiffness 0);
 * the rest get `stiffness` (default 1.0, the NAOqi convention) or the commanded kp if
 * `use_command_kp` is true.
 *
 * In a simulator, which closes a PD on the command, `servo: {kp, kd}` stands for the servo
 * of the robot: the output is then kp = stiffness * servo.kp [Nm/rad] and
 * kd = stiffness * servo.kd [Nm s/rad] (a stiffness 0..1 scales the servo like NAOqi does).
 * Without `servo` the output keeps the NAOqi convention, for the robot's own servos.
 */
class PositionServo : public LimitsBase
{
public:
  void update(
    const gaitos::JointState & measured, const gaitos::JointCommand & command,
    gaitos::JointCommand & output) override;

protected:
  bool on_initialize() override;

private:
  double stiffness_{1.0};
  bool use_command_kp_{false};
  double servo_kp_{0.0};   ///< > 0: simulated servo
  double servo_kd_{0.0};
};

}  // namespace gaitos_actuators

#endif  // GAITOS_ACTUATORS__ACTUATORS_HPP_
