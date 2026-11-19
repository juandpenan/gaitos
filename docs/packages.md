# Packages

[← gaitos](../README.md#documentation)

The ament packages of the repository (colcon), from the bottom layer up.

| Package | Contents |
|---|---|
| `gaitos` | Metapackage: `params/gaitos.yaml` (common parameters), `params/fake.yaml`, `launch/gaitos.launch.py`. |
| `gaitos_msgs` | `srv/SetMode`, `msg/Status`. |
| `gaitos_common` | Types (`JointCommand`, `ImuData`, `Odometry`, `BaseState`, `ControlReference`...), `RobotState` (blackboard of the cycle), `Params` (view of the ROS 2 parameters, `package://` URIs), logging through rclcpp. |
| `gaitos_model` | URDF and SRDF parsing, `RobotModel` (joints in hardware order, legs, named poses), kinematics: forward, jacobians, damped least squares inverse kinematics, mimic joints, composite rigid body. |
| `gaitos_core` | Plugin interfaces (`HardwareInterface`, `JointActuatorBase`, `ImuSensorBase`, `ContactSensorBase`, `StateEstimatorBase`, `ControllerBase`, `OperationModeBase`), `PluginLoader` (pluginlib), gains. |
| `gaitos_hardware` | `FakeHardware` plugin: no physics, for tests and display. |
| `gaitos_actuators` | Motor plugins: `PdPassthrough`, `TorquePd`, `PositionServo`. |
| `gaitos_sensors` | `ImuPassthrough`, `ComplementaryImu`, `ForceThresholdContact`, `TorqueContact`. |
| `gaitos_estimators` | `KalmanEstimator`, `LegOdometryEstimator`, `ExternalOdometryEstimator`. |
| `gaitos_controllers` | Kinematic controllers: `JointPdController`, `StanceController`, `GaitController`. |
| `gaitos_dynamics` | Dynamic controllers: `LipmZmpController`, `ConvexMpcController` (OSQP). |
| `gaitos_modes` | `ModeManager` and the mode plugins: `ZeroTorqueMode`, `DampMode`, `PostureMode`, `BalanceStandMode`, `LocomotionMode`. |
| `gaitos_system` | `Gaitos` (the cycle and its safety), `GaitosNode` (lifecycle node) and the `gaitos_node` executable. |
| `gaitos_description` | `robots/<robot>_description/{urdf, srdf, config, mjcf}` (Go2, R1, NAO), `launch/display.launch.py`, RViz configuration, `scripts/make_gazebo_urdf.py` (physics URDF variant), `scripts/make_mjcf.py` (MJCF from it); fetches the Unitree meshes and MuJoCo assets (FetchContent) and, with `GAITOS_NAO_MESHES`, the NAO meshes. |
| `gaitos_mujoco` | `MujocoHardware` plugin, `launch/sim.launch.py`, `params/`, `snapshot` tool. |
| `gaitos_gazebo` | `gaitos_gz_system` plugin (hosts the node in gz-sim), worlds, `launch/sim.launch.py`, `params/`. |
| `gaitos_unitree` | `UnitreeGoHardware` (Go2) and `UnitreeHgHardware` (R1) plugins over unitree_sdk2 (FetchContent), `launch/robot.launch.py`, `params/`, `go2_dds_sim` tool. |
| `gaitos_naoqi_sim` | `NaoqiDriverEmulator` extension: the ROS 2 interface of naoqi_driver (`/joint_states`, `/odom`, `/cmd_vel`, `/joint_angles`, `/move_base_simple/goal`...) in front of a simulated gaitos NAO; `launch/sim.launch.py`, `params/nao_naoqi_sim.yaml`. |
| `naoqi_msgs/` | `naoqi_bridge_msgs`: the naoqi_driver interfaces (ros-naoqi naoqi_bridge_msgs2), fetched with FetchContent and generated here. |
| `gaitos_unitree_sim` | `UnitreeHgEmulator` (R1) and `UnitreeGoEmulator` (Go2) extensions: the DDS interface of the robot (`/lowstate`, `/sportmodestate`, `/api/sport`...) in front of a simulated gaitos, for programs written for the real robot; `launch/sim.launch.py`, `params/<robot>_unitree_sim.yaml`. |
| `unitree_ros2_msgs/` | `unitree_api`, `unitree_go`, `unitree_hg`: the unitree_ros2 messages, fetched with FetchContent and generated here. They are not needed by gaitos; they let `ros2 topic` read the robot topics. |
