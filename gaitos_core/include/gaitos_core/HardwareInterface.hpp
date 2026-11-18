//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CORE__HARDWAREINTERFACE_HPP_
#define GAITOS_CORE__HARDWAREINTERFACE_HPP_

#include <chrono>

#include "gaitos_common/RobotState.hpp"
#include "gaitos_core/PluginBase.hpp"

namespace gaitos
{

/**
 * @class HardwareInterface
 * @brief Backend that connects gaitos to a robot or a simulator (Unitree SDK2, MuJoCo,
 * Gazebo, a fake robot...).
 *
 * Each cycle the system calls read(), runs sensors, estimator, mode and controller, and
 * calls write() with the command of every actuated joint. Simulated backends advance the
 * simulation in write() (lockstep) and report the simulated time in now().
 */
class HardwareInterface : public PluginBase
{
public:
  /**
   * @brief Connect to the robot / load the simulation. Called after initialize().
   */
  virtual bool start() {return true;}

  /**
   * @brief Disconnect. The robot must be left in a safe state (passive or damped).
   */
  virtual void stop() {}

  /**
   * @brief Read the sensors: state.joints, state.imu_raw, state.foot_force_raw (if the robot
   * has foot sensors), state.remote (if it has a hand held remote) and the external
   * odometries it provides (e.g. "ground_truth")
   * @return false if there is no valid data (the system goes to its safe mode)
   */
  virtual bool read(RobotState & state) = 0;

  /**
   * @brief Send the joint commands (tau = kp (q* - q) + kd (dq* - dq) + tau_ff)
   */
  virtual bool write(const JointCommand & command) = 0;

  /**
   * @brief Current time [s]: steady clock on a robot, simulated time in a simulator
   */
  virtual double now() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  /**
   * @brief Leave the robot safe NOW (damped or passive joints) without going through the
   * cycle. Called from the watchdog thread when the cycle stops running, so it must be
   * thread safe and must not block. Real robot backends should implement it.
   */
  virtual void emergency_stop() {}

  virtual bool is_simulation() const {return false;}

  /**
   * @brief Whether the backend dictates the pace of the loop (write() blocks until the
   * next cycle). A lockstep simulator running as fast as possible returns true.
   */
  virtual bool paces_loop() const {return false;}

  /**
   * @brief Whether the backend may send anything to the robot. The system clears it in
   * dry runs; backends that send commands outside write() (stop(), emergency_stop()) must
   * check it.
   */
  bool write_enabled() const {return write_enabled_;}
  void set_write_enabled(bool enabled) {write_enabled_ = enabled;}

private:
  bool write_enabled_{true};
};

}  // namespace gaitos

#endif  // GAITOS_CORE__HARDWAREINTERFACE_HPP_
