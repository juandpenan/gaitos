//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_HARDWARE__FAKEHARDWARE_HPP_
#define GAITOS_HARDWARE__FAKEHARDWARE_HPP_

#include <atomic>
#include <mutex>
#include <vector>

#include "gaitos_core/HardwareInterface.hpp"

namespace gaitos_hardware
{

/**
 * @class FakeHardware
 * @brief Robot without physics for tests and dry runs: the base stays where it is, each
 * joint is a unit inertia driven by the commanded torque, the IMU is level and the feet
 * report a constant force. Time is simulated: each write() advances `period` seconds.
 *
 * Parameters: period [s] (0.002), joint_inertia (0.05), joint_damping (0.5),
 * initial_pose (SRDF pose, optional), foot_force (100), base_height (0.3),
 * publish_ground_truth (true).
 */
class FakeHardware : public gaitos::HardwareInterface
{
public:
  bool read(gaitos::RobotState & state) override;
  bool write(const gaitos::JointCommand & command) override;
  double now() const override {return time_;}
  bool is_simulation() const override {return true;}
  bool paces_loop() const override {return true;}

  void emergency_stop() override {++emergency_stops_;}

  /// Last command received (tests)
  const gaitos::JointCommand & last_command() const {return command_;}
  /// Number of write() calls (tests)
  uint64_t writes() const {return writes_;}
  /// Number of emergency_stop() calls (tests)
  int emergency_stops() const {return emergency_stops_;}
  /// Simulated remote (tests); stamped with the hardware time
  void set_remote(const gaitos::RemoteInput & remote);
  /// Block the next read() for that wall time (watchdog tests)
  void stall(double seconds) {stall_ = seconds;}

protected:
  bool on_initialize() override;

private:
  double period_{0.002};
  double inertia_{0.05};
  double damping_{0.5};
  double foot_force_{100.0};
  double base_height_{0.3};
  bool ground_truth_{true};
  double time_{0.0};
  uint64_t seq_{0};
  std::vector<double> q_;
  std::vector<double> dq_;
  std::vector<double> tau_;
  gaitos::JointCommand command_;
  uint64_t writes_{0};
  std::atomic<int> emergency_stops_{0};
  std::atomic<double> stall_{0.0};
  std::mutex remote_mutex_;
  gaitos::RemoteInput remote_;
};

}  // namespace gaitos_hardware

#endif  // GAITOS_HARDWARE__FAKEHARDWARE_HPP_
