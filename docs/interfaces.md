# ROS 2 interfaces

[← gaitos](../README.md#documentation)

One `gaitos_node` per robot, named `gaitos`, in the namespace of the robot (`/go2`...).

| Kind | Name | Type | |
|---|---|---|---|
| Subscription | `cmd_vel` | `geometry_msgs/Twist` | Body twist (x forward, y left, yaw rate). Refresh at ≥ 2 Hz; it stops after `command_timeout`. |
| Subscription | `goal_pose` | `geometry_msgs/PoseStamped` | Relative to the body (`frame_id` the base frame or empty) or in `odom`. Standing: body posture; walking: target. |
| Subscription | `external_odometry.topics[i]` | `nav_msgs/Odometry` | External odometry `external_odometry.sources[i]`, fused by the estimator. |
| Subscription | `joy` | `sensor_msgs/Joy` | Remote (`teleop.joy.enabled`). |
| Subscription | `joint_targets` | `sensor_msgs/JointState` | Positions (and max speeds in `velocity`) of joints moved over the controller: the joints of `external_joints.groups`, any joint in `external_joints.any_joint_modes`; a mode change drops them. |
| Service | `set_mode` | `gaitos_msgs/SetMode` | Request a mode. |
| Service | `mode/<name>` | `std_srvs/Trigger` | One per mode. |
| Publisher | `mode` | `std_msgs/String` | Active mode (transient local). |
| Publisher | `status` | `gaitos_msgs/Status` | Mode, controller, contacts, foot forces, dry run, last safety event. |
| Publisher | `odometry` | `nav_msgs/Odometry` | Estimate of the base (`odom` → base frame). |
| Publisher | `joint_states` | `sensor_msgs/JointState` | For `robot_state_publisher`. |
| Publisher | `ground_truth` | `nav_msgs/Odometry` | Simulators. |
| TF | `odom` → base | | `publish_tf`; frames `frames.odom`, `frames.base`, `frames.prefix`. |
| Publisher | `/clock` | `rosgraph_msgs/Clock` | MuJoCo (`publish_clock`). |

The node is a lifecycle node: `gaitos_node` configures and activates itself (`autostart`,
default true); configure loads the model and every plugin, activate starts the hardware and
the real-time loop thread, deactivate leaves the robot in the safe mode.

`publish.joint_states`, `publish.odometry` and `publish_tf` (default true) turn off the robot
state when another stack publishes it. Extensions (`gaitos::NodeExtension`, pluginlib) add
more ROS interfaces to the node in front of the same gaitos, configured as
`extensions.<name>.plugin` plus their own parameters (e.g. `gaitos_unitree_sim`).
