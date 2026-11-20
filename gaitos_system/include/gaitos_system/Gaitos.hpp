//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_SYSTEM__GAITOS_HPP_
#define GAITOS_SYSTEM__GAITOS_HPP_

#include <atomic>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "gaitos_common/Params.hpp"
#include "gaitos_common/RobotState.hpp"
#include "gaitos_core/ControllerBase.hpp"
#include "gaitos_core/HardwareInterface.hpp"
#include "gaitos_core/JointActuatorBase.hpp"
#include "gaitos_core/PluginLoader.hpp"
#include "gaitos_core/SensorBase.hpp"
#include "gaitos_core/StateEstimatorBase.hpp"
#include "gaitos_modes/ModeManager.hpp"
#include "gaitos_system/Safety.hpp"

namespace gaitos
{

/**
 * @struct Snapshot
 * @brief Copy of the state at the end of a cycle, for other threads (console, ROS
 * wrappers, loggers).
 */
struct Snapshot
{
  double time{0.0};
  uint64_t cycle{0};
  std::string mode;
  std::string controller;
  BaseState base;
  ImuData imu;
  ContactData contacts;
  JointState joints;
  JointCommand command;
  std::map<std::string, Odometry> external_odometry;
  bool hardware_ok{false};
  bool dry_run{false};
  std::string last_safety_event;
  bool simulation{false};
};

/**
 * @struct JointTarget
 * @brief Position target of one joint from outside the controller (e.g. a head or an arm
 * moved by an application while the robot stands or walks).
 */
struct JointTarget
{
  std::string name;
  double position{0.0};    ///< [rad], or a change of the current target if `relative`
  double speed{0.0};       ///< [rad/s]; <= 0: the max speed of the joint (URDF)
  bool relative{false};
};

/**
 * @struct JointCommandOverride
 * @brief Full motor command of one joint from outside the controller (e.g. the arm_sdk of a
 * Unitree humanoid): position, velocity, feedforward torque and gains, blended with the output
 * of the controller by `weight`.
 */
struct JointCommandOverride
{
  std::string name;
  double position{0.0};    ///< [rad]
  double velocity{0.0};    ///< [rad/s]
  double effort{0.0};      ///< Feedforward torque [Nm]
  double kp{0.0};          ///< [Nm/rad]
  double kd{0.0};          ///< [Nm s/rad]
  double weight{1.0};      ///< 0: the controller's command, 1: this one
};

/**
 * @class Gaitos
 * @brief The gaitos system: loads the robot model and every plugin from the configuration
 * and runs the cycle
 *
 *   hardware.read -> imu -> contacts -> estimator -> mode -> controller -> actuators -> hardware.write
 *
 * Safety: joint limits (safety.limits), gain ramp when leaving a passive mode
 * (safety.gain_ramp_time, safety.ramp_from), hand held remote with emergency stop
 * (teleop), watchdog of the cycle in run() (safety.watchdog_timeout: the hardware
 * emergency_stop() is called if no cycle completes in that time) and `dry_run` (the
 * hardware is read but never written, to check a real robot safely).
 *
 * It does not depend on ROS. The cycle runs in one thread (run() or the host calling
 * step(), e.g. the Gazebo plugin); the command methods (request_mode, set_twist,
 * set_pose_goal, push_odometry) and snapshot() can be called from any thread: the
 * commands are queued and applied at the start of the next cycle.
 */
class Gaitos
{
public:
  Gaitos();
  ~Gaitos();

  /**
   * @brief Build everything from the ROS 2 parameters of the node (root block)
   * @param hardware backend instance to use (not initialized yet) instead of creating the
   * `hardware.plugin` one; hosts like the Gazebo plugin provide their own
   * @throws std::runtime_error / ParamsError on errors
   */
  void configure(const Params & config, std::shared_ptr<HardwareInterface> hardware = nullptr);

  /**
   * @brief Start the hardware
   */
  bool start();

  /**
   * @brief Leave the robot in the safe mode and stop the hardware
   */
  void stop();

  /**
   * @brief Run one cycle
   * @return false if the hardware failed in this cycle
   */
  bool step();

  /**
   * @brief Run cycles at `frequency` Hz (or as fast as the hardware allows if it paces the
   * loop) until `running` is false, applying the real time configuration
   */
  void run(const std::atomic<bool> & running);

  // Commands (thread safe)
  std::future<CommandResult> request_mode(const std::string & mode);
  void set_twist(const Eigen::Vector3d & linear, const Eigen::Vector3d & angular);
  void set_pose_goal(const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation);
  void push_odometry(const std::string & source, const Odometry & odometry);
  /// Hand held remote from outside the hardware (e.g. sensor_msgs/Joy); replaces the one of
  /// the hardware while it is newer than `teleop.timeout`
  void set_remote(const RemoteInput & remote);
  /**
   * @brief Position targets for some joints, over the output of the controller: each one
   * moves toward its target at its speed and holds it (the gains stay the controller ones).
   * Only the joints of `external_joints.groups` (SRDF groups), or any joint in the modes of
   * `external_joints.any_joint_modes`; never in the passive modes
   * (`external_joints.off_modes`, default zero_torque and the safe mode). A mode change drops
   * every target. Unknown or not allowed joints are ignored.
   * @return number of joints accepted
   */
  std::size_t set_joint_targets(const std::vector<JointTarget> & targets);
  /// Joints that may take external targets in the current mode
  bool external_joint_allowed(std::size_t joint, const std::string & mode) const;
  /**
   * @brief Full commands for some joints, blended over the output of the controller (and the
   * external targets): command = (1 - weight) * controller + weight * override, for the
   * position, velocity, torque and gains. Same joints as set_joint_targets()
   * (external_joint_allowed()). Commands with a non-finite value are ignored; the position is
   * limited to the joint range, the velocity and torque to the joint limits (URDF), the gains
   * to >= 0 and the weight to [0, 1]. Each joint keeps its last command until a mode change or
   * until none arrives for `external_joints.command_timeout` seconds (default 0.5); then its
   * weight ramps down to 0 over `external_joints.release_time` seconds (default 0.2). A new
   * command (e.g. a sender that keeps streaming over a mode change) takes it again at once. In a
   * passive mode (`external_joints.off_modes`) the joint is given back at once.
   * Thread safe; no ROS interface (vendor emulators call it).
   * @return number of joints with a known name
   */
  std::size_t set_joint_commands(const std::vector<JointCommandOverride> & commands);

  /**
   * @brief Called (in the cycle thread) on every mode change
   */
  void set_mode_callback(ModeManager::ChangeCallback cb);

  Snapshot snapshot() const;

  RobotModelPtr model() const {return model_;}
  const RobotState & state() const {return state_;}   ///< Cycle thread only
  const Params & config() const {return config_;}
  double frequency() const {return frequency_;}
  std::vector<std::string> modes() const {return modes_.modes();}
  HardwareInterface & hardware() {return *hardware_;}
  StateEstimatorBase & estimator() {return *estimator_;}
  ControllerBase * controller(const std::string & name) const;

private:
  void apply_commands();
  void apply_external_joints();
  void apply_command_overrides();

  // pluginlib loaders: declared first, destroyed after the plugins they created
  PluginLoader<HardwareInterface> hardware_loader_{"gaitos::HardwareInterface"};
  PluginLoader<JointActuatorBase> actuator_loader_{"gaitos::JointActuatorBase"};
  PluginLoader<ImuSensorBase> imu_loader_{"gaitos::ImuSensorBase"};
  PluginLoader<ContactSensorBase> contact_loader_{"gaitos::ContactSensorBase"};
  PluginLoader<StateEstimatorBase> estimator_loader_{"gaitos::StateEstimatorBase"};
  PluginLoader<ControllerBase> controller_loader_{"gaitos::ControllerBase"};

  Params config_;
  RobotModelPtr model_;
  RobotState state_;
  std::shared_ptr<HardwareInterface> hardware_;
  std::vector<std::shared_ptr<JointActuatorBase>> actuators_;
  std::shared_ptr<ImuSensorBase> imu_;
  std::shared_ptr<ContactSensorBase> contacts_;
  std::shared_ptr<StateEstimatorBase> estimator_;
  std::map<std::string, std::shared_ptr<ControllerBase>> controllers_;
  ModeManager modes_;
  Logger logger_{"gaitos"};

  double frequency_{500.0};
  int max_hardware_failures_{10};
  int hardware_failures_{0};
  bool started_{false};
  bool first_cycle_{true};
  double last_time_{0.0};
  double real_time_factor_{1.0};
  uint64_t overruns_{0};

  // Safety
  bool dry_run_{false};
  SafetyMonitor safety_;
  Teleop teleop_;
  double gain_ramp_time_{0.0};
  std::vector<std::string> ramp_from_;
  double ramp_start_{-1.0};
  std::string previous_mode_;
  double watchdog_timeout_{0.0};
  std::atomic<double> last_cycle_wall_{0.0};
  std::string last_safety_event_;

  // External joint targets
  std::vector<bool> external_group_;            ///< joint in external_joints.groups
  std::vector<std::string> external_any_modes_;
  std::vector<std::string> external_off_modes_;
  struct ExternalJoint
  {
    bool active{false};
    double target{0.0};
    double speed{0.0};
    double setpoint{0.0};
  };
  std::vector<ExternalJoint> external_;
  std::string external_mode_;
  std::vector<JointTarget> external_inbox_;
  struct CommandOverride
  {
    bool active{false};
    bool releasing{false};   ///< blending out toward the controller
    JointCommandOverride command;
    double weight{0.0};      ///< current weight (the command's, or ramping down)
    double stamp{0.0};
  };
  std::vector<CommandOverride> overrides_;
  std::string override_mode_;
  std::vector<JointCommandOverride> override_inbox_;
  double override_timeout_{0.5};
  double override_release_time_{0.2};

  // Command inbox
  mutable std::mutex inbox_mutex_;
  std::vector<std::pair<std::string, std::promise<CommandResult>>> mode_requests_;
  bool has_twist_{false};
  BodyTwist twist_;
  bool has_pose_goal_{false};
  BodyPoseGoal pose_goal_;
  std::map<std::string, Odometry> odometry_inbox_;
  bool has_remote_{false};
  RemoteInput remote_inbox_;
  RemoteInput remote_;

  mutable std::mutex snapshot_mutex_;
  Snapshot snapshot_;
};

}  // namespace gaitos

#endif  // GAITOS_SYSTEM__GAITOS_HPP_
