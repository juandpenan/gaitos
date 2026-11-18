//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_UNITREE__UNITREEHARDWARE_HPP_
#define GAITOS_UNITREE__UNITREEHARDWARE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "gaitos_core/HardwareInterface.hpp"

namespace gaitos_unitree
{

/**
 * @class UnitreeHardwareBase
 * @brief Low level control of Unitree robots through unitree_sdk2 (DDS): reads rt/lowstate
 * and writes rt/lowcmd with the joint commands (q, dq, kp, kd, tau: the motor drivers close
 * the PD). Works with the real robot and with the unitree_mujoco simulator (domain_id 1,
 * network_interface lo).
 *
 * @code{.yaml}
 * hardware:
 *   plugin: gaitos_unitree/UnitreeGoHardware   # Go2, Go2W, B2 (unitree_go IDL)
 *                                              # gaitos_unitree/UnitreeHgHardware: R1, G1, H1-2
 *   network_interface: eth0      # "lo" for unitree_mujoco
 *   domain_id: 0                 # 1 for unitree_mujoco
 *   motor_index: {}              # gaitos joint -> motor index (default: joint order)
 *   release_sport_mode: true     # release the built-in sport/loco controller first
 *                                # (real robot; it would fight the low level commands)
 *   state_timeout: 0.1           # [s] max age of rt/lowstate
 *   foot_force_scale: 1.0        # Go2 foot_force units -> gaitos units
 *   sport_odometry: false        # Go2: rt/sportmodestate as external odometry "unitree_sport"
 *   emergency_kd: 3.0            # damping sent by emergency_stop()
 * @endcode
 *
 * The wireless controller (LowState wireless_remote) is read into state.remote, for the
 * teleop block of the system (emergency stop, modes, sticks). emergency_stop() sends damped
 * joints from the calling thread (the watchdog of the system).
 */
class UnitreeHardwareBase : public gaitos::HardwareInterface
{
public:
  ~UnitreeHardwareBase() override = default;

protected:
  bool on_initialize() override;

  /// Initialize the DDS channel factory once per process
  void init_channels();
  /// Release the built-in motion service of the robot
  void release_motion_service();
  bool state_fresh() const;
  /// Parse the 40 bytes of the wireless remote of LowState
  static gaitos::RemoteInput parse_remote(const uint8_t * data, double stamp);

  std::string interface_{"eth0"};
  int domain_id_{0};
  std::vector<int> motor_index_;
  bool release_sport_mode_{true};
  double state_timeout_{0.1};
  double foot_force_scale_{1.0};
  bool sport_odometry_{false};
  double emergency_kd_{3.0};

  mutable std::mutex mutex_;
  std::atomic<double> last_state_{-1.0};
};

class UnitreeGoHardware : public UnitreeHardwareBase
{
public:
  UnitreeGoHardware();
  ~UnitreeGoHardware() override;
  bool start() override;
  void stop() override;
  bool read(gaitos::RobotState & state) override;
  bool write(const gaitos::JointCommand & command) override;
  void emergency_stop() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class UnitreeHgHardware : public UnitreeHardwareBase
{
public:
  UnitreeHgHardware();
  ~UnitreeHgHardware() override;
  bool start() override;
  void stop() override;
  bool read(gaitos::RobotState & state) override;
  bool write(const gaitos::JointCommand & command) override;
  void emergency_stop() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gaitos_unitree

#endif  // GAITOS_UNITREE__UNITREEHARDWARE_HPP_
