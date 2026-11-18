//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_COMMON__ROBOTSTATE_HPP_
#define GAITOS_COMMON__ROBOTSTATE_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "gaitos_common/Types.hpp"

namespace gaitos
{

/**
 * @struct ExternalOdometry
 * @brief Last message of an external odometry source.
 */
struct ExternalOdometry
{
  Odometry odometry;
  double reception{0.0};   ///< Reception time [s]
  uint64_t seq{0};         ///< Increased on every message (0: none yet)
};

/**
 * @class RobotState
 * @brief Blackboard shared by every stage of the gaitos cycle.
 *
 * The system owns one instance and passes it through the stages of each cycle, all in the
 * same thread, so no locking is needed. Every field has a single writer:
 *  - joints, imu_raw, foot_force_raw, remote, external_odometry (backend part): hardware
 *  - external_odometry (API part), twist_command, pose_goal: system (command inbox)
 *  - imu: IMU sensor plugin
 *  - contacts: contact sensor plugin
 *  - base: state estimator
 *  - mode, reference: mode manager / active mode
 *  - command: active controller
 *  - motor_command: actuator plugins (what goes to the hardware)
 */
struct RobotState
{
  void resize(std::size_t joints, std::size_t feet);

  std::string name{"gaitos"};
  double time{0.0};          ///< Time of the current cycle [s]
  double dt{0.0};            ///< Time since the previous cycle [s]
  uint64_t cycle{0};         ///< Cycle counter
  bool simulation{false};

  // Hardware
  JointState joints;
  ImuData imu_raw;
  std::vector<double> foot_force_raw;   ///< Per foot, empty if there are no foot sensors
  RemoteInput remote;                   ///< Hand held remote, if the hardware has one
  std::map<std::string, ExternalOdometry> external_odometry;

  // Sensors and estimation
  ImuData imu;
  ContactData contacts;
  BaseState base;

  // Commands from the user
  BodyTwist twist_command;
  BodyPoseGoal pose_goal;
  uint64_t pose_goal_seq{0};    ///< Increased on every new pose goal

  // Modes and control
  std::string mode;
  std::string controller;
  ControlReference reference;
  JointCommand command;          ///< Controller output
  JointCommand motor_command;    ///< Sent to the hardware
};

}  // namespace gaitos

#endif  // GAITOS_COMMON__ROBOTSTATE_HPP_
