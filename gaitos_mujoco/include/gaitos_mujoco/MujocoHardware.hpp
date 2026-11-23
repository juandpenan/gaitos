//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_MUJOCO__MUJOCOHARDWARE_HPP_
#define GAITOS_MUJOCO__MUJOCOHARDWARE_HPP_

#include <mujoco/mujoco.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "gaitos_core/HardwareInterface.hpp"

namespace gaitos_mujoco
{

/**
 * @class MujocoHardware
 * @brief MuJoCo simulation inside the gaitos process, in lockstep: each write() applies the
 * joint commands and advances `period` seconds of simulation.
 *
 * The motors are simulated as the Unitree drivers: the PD of the command
 * (tau = kp (q* - q) + kd (dq* - dq) + tau_ff) is closed at every physics step and the
 * torque is limited to the URDF effort. It goes to the motor actuator of the joint if the
 * model has one, else it is applied as a generalized force.
 *
 * @code{.yaml}
 * hardware:
 *   plugin: gaitos_mujoco/MujocoHardware
 *   model: package://gaitos_description/robots/go2_description/mjcf/scene.xml   # MJCF or URDF
 *   world_includes: []         # MJCF files whose top-level bodies (with their geoms, lights,
 *                              # free joints and assets) are added to the world of the model:
 *                              # a room, furniture, objects
 *   period: 0.002              # [s] simulated time per cycle (1 / frequency)
 *   timestep: 0.0              # physics step [s], 0: the one of the model
 *   base_body: base_link       # body of the floating base
 *   initial_pose: lie          # SRDF pose at start (empty: keyframe or model defaults)
 *   keyframe: ""               # MJCF keyframe at start
 *   initial_height: -1.0       # [m] base height, < 0: lowest geometry touching the floor
 *   joint_map: {}              # gaitos joint -> MuJoCo joint, when the names differ
 *   imu: {orientation: imu_quat, gyro: imu_gyro, accelerometer: imu_acc}  # sensor names;
 *                              # missing sensors are computed from the base body
 *   feet:                      # per leg: geoms (names) and/or bodies whose contacts count
 *     FR: {geoms: [FR]}
 *   hold_gains: {kp: 50.0, kd: 2.0}   # PD that holds simulated joints gaitos does not know
 *                              # (auxiliary joints: command_auxiliary() moves them, with its
 *                              # own gains; their torque is limited to actuatorfrcrange)
 *   ground_truth: true         # publish the "ground_truth" external odometry
 * @endcode
 */
class MujocoHardware : public gaitos::HardwareInterface
{
public:
  ~MujocoHardware() override;

  bool start() override;
  void stop() override;
  bool read(gaitos::RobotState & state) override;
  bool write(const gaitos::JointCommand & command) override;
  double now() const override {return data_ ? data_->time : 0.0;}
  bool is_simulation() const override {return true;}
  bool paces_loop() const override {return true;}
  bool command_auxiliary(
    const std::vector<std::string> & names, const std::vector<double> & position,
    const std::vector<double> & kp, const std::vector<double> & kd) override;
  bool read_auxiliary(
    const std::vector<std::string> & names, std::vector<double> & position,
    std::vector<double> & velocity, std::vector<double> & effort) const override;

  /// Simulation access (tests, tools; not thread safe, see render_state() for other threads)
  const mjModel * mj_model() const {return model_;}
  mjData * mj_data() const {return data_;}

  /// State to render the simulation from another thread (the model does not change once
  /// started): positions (qpos), mocap poses and simulated time after the last write()
  struct RenderState
  {
    std::vector<mjtNum> qpos;
    std::vector<mjtNum> mocap_pos;
    std::vector<mjtNum> mocap_quat;
    double time{0.0};
    uint64_t step{0};          ///< 0: not started
  };
  /// Copy the latest state into `state` if it is newer than `state.step`. Thread safe.
  /// @return whether it was copied
  bool render_state(RenderState & state) const;

protected:
  bool on_initialize() override;

private:
  struct JointMap
  {
    int qpos{-1};        ///< -1: the joint is not in the simulation
    int dof{-1};
    int actuator{-1};
    double gear{1.0};
  };
  /// Simulated joint that mimics a model joint (URDF <mimic>): it follows the command of its
  /// master, q = multiplier * q_master + offset, with the master gains
  struct MimicJoint
  {
    int qpos;
    int dof;
    int actuator;
    double gear;
    std::size_t master;
    double multiplier;
    double offset;
  };
  /// Simulated joint gaitos does not control: held where it starts, or an auxiliary joint
  /// moved by command_auxiliary()
  struct HeldJoint
  {
    int qpos;
    int dof;
    int actuator;
    double gear;
    double target;
    double kp{-1.0};          ///< < 0: hold_gains
    double kd{-1.0};
    double limit{-1.0};       ///< [Nm or N] < 0: no limit
    double effort{0.0};       ///< last applied
  };
  struct AuxiliaryCommand
  {
    double position;
    double kp;
    double kd;
  };
  struct AuxiliaryState
  {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
  };

  bool load_model();
  void place_robot();
  void apply_torques();
  void update_auxiliary_state();
  void update_render_state();
  int sensor_adr(const std::string & name, int dim) const;

  mjModel * model_{nullptr};
  mjData * data_{nullptr};
  std::string file_;
  double period_{0.002};
  int substeps_{1};
  int base_body_{-1};
  int base_qpos_{-1};
  int base_dof_{-1};
  std::vector<JointMap> joints_;
  std::vector<MimicJoint> mimic_;
  std::vector<HeldJoint> held_;
  mutable std::mutex render_mutex_;
  RenderState render_state_;
  std::map<std::string, std::size_t> held_index_;     ///< joint name -> held_
  mutable std::mutex auxiliary_mutex_;
  std::map<std::size_t, AuxiliaryCommand> auxiliary_commands_;  ///< held_ -> latest, applied in write()
  std::vector<AuxiliaryState> auxiliary_state_;       ///< per held_, after the last write()
  double hold_kp_{50.0};
  double hold_kd_{2.0};
  int imu_quat_{-1};
  int imu_gyro_{-1};
  int imu_acc_{-1};
  Eigen::Vector3d last_velocity_{Eigen::Vector3d::Zero()};
  std::vector<std::vector<int>> foot_geoms_;    ///< per leg
  std::vector<int> geom_foot_;                  ///< geom -> leg (-1)
  bool ground_truth_{true};
  uint64_t seq_{0};
  gaitos::JointCommand command_;
  std::vector<double> fake_q_;                  ///< Joints missing in the simulation
};

}  // namespace gaitos_mujoco

#endif  // GAITOS_MUJOCO__MUJOCOHARDWARE_HPP_
