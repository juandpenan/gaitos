# Parameters

[← gaitos](../README.md#documentation)

Launch files load, in order (later files override earlier ones):

1. `gaitos/params/gaitos.yaml`: loop, real time, defaults of the plugins, safety;
2. `gaitos_description/robots/<robot>_description/config/<robot>.yaml`: model, plugins,
   controllers and modes of the robot;
3. the backend file: `gaitos_mujoco/params/<robot>_mujoco.yaml`,
   `gaitos_gazebo/params/<robot>_gazebo.yaml`, `gaitos_unitree/params/<robot>_unitree*.yaml`,
   `gaitos/params/fake.yaml`;
4. `robot_description` and `robot_description_semantic` (URDF and SRDF text), as for
   `robot_state_publisher` and MoveIt; without them, `robot.urdf` / `robot.srdf` paths or
   `package://` URIs are read.

| Block | Contents |
|---|---|
| `robot` | `name`, `base_link`, `joints` (order), `legs.names`, `legs.groups`, `legs.orientation_ik`. |
| `frequency`, `real_time`, `real_time_factor` | Loop rate; SCHED_FIFO, memory locking, CPU affinity; simulation speed. |
| `hardware` | Backend plugin and its parameters. |
| `actuators.<name>` | Actuator plugins, each with its `joints` (names, `all`, `legs`, `upper_body` or SRDF groups). |
| `imu`, `contacts`, `estimator` | One plugin each. |
| `controllers.<name>` | Controllers (`joint` is always there). |
| `modes.<name>`, `initial_mode` | Operation modes. |
| `safety`, `teleop`, `dry_run` | Safety ([Safety and the real robot](safety.md)). |
| `external_odometry.sources`, `.topics` | External odometry topics. |
| `external_joints.groups`, `.any_joint_modes`, `.off_modes` | Joints applications move over the controller: position targets (`joint_targets`, `Gaitos::set_joint_targets`) and full motor commands blended by a weight (`Gaitos::set_joint_commands`, used by vendor emulators such as the Unitree arm_sdk; no ROS interface). |
| `external_joints.command_timeout`, `.release_time` | `set_joint_commands` only: when the sender stops (no command for `command_timeout` s, 0.5) or the mode changes, the joint is blended back to the controller over `release_time` s (0.2); a new command takes it again at its own weight (arm_sdk clients ramp the weight themselves). |
| `frames.*`, `publish_tf`, `publish_rate`, `publish_clock` | ROS outputs. |

Gains are given per joint, per group or by default (`default`, then groups, then joints):
`gains: {default: {kp: 40, kd: 1}, legs: {kp: 60, kd: 3}, FR_calf_joint: {kp: 80}}`.
ROS 2 parameters have no lists of maps, so sequences are parallel lists (`legs.names` /
`legs.groups`, `poses` / `durations`, `teleop.buttons` / `teleop.modes`).
