# Go2: the real robot and its DDS simulator

[← Tutorials](README.md)

The real Unitree Go2 through `unitree_sdk2` (low level: `rt/lowstate`, `rt/lowcmd` with its
CRC), from the `gaitos_unitree/UnitreeGoHardware` plugin. **Not tested on the real robot yet**:
follow the staged sequence below, with the robot hanging or lying first.

## 1. Before starting

```bash
cd ~/ws/legged_ws
source install/setup.bash
export ROS_DOMAIN_ID=10      # NOT 0 with the robot connected (see "CycloneDDS" below)
```

`unitree_sdk2` is fetched when `gaitos_unitree` is built. The PC is wired to the robot network
(192.168.123.0/24).

## 2. First: the DDS simulator (no robot)

`go2_dds_sim` is a headless Go2 (MuJoCo) speaking the same DDS topics as the robot, on the
interface `lo` and domain 1:

```bash
ros2 run gaitos_unitree go2_dds_sim \
  $(ros2 pkg prefix gaitos_description)/share/gaitos_description/robots/go2_description/mjcf/scene.xml
ros2 launch gaitos_unitree robot.launch.py robot:=go2 dds_sim:=true
ros2 service call /go2/mode/stand_up std_srvs/srv/Trigger
ros2 service call /go2/mode/balance_stand std_srvs/srv/Trigger
ros2 topic pub -r 10 /go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

The same works against `unitree_mujoco` (its simulator configured with `robot: go2`,
`domain_id: 1`, `interface: lo`).

## 3. The robot

```bash
ros2 launch gaitos_unitree robot.launch.py robot:=go2 readonly:=true network_interface:=eth0   # 1) read only
ros2 launch gaitos_unitree robot.launch.py robot:=go2 network_interface:=eth0 [joy:=true]      # 2) commanding
```

| Argument | Default | |
|---|---|---|
| `readonly` | `false` | `dry_run`: everything runs but the hardware is never written |
| `network_interface` | from the parameters | the NIC wired to the robot |
| `dds_sim` | `false` | `lo`, domain 1 (the DDS simulators) |
| `joy` | `false` | also a `joy_node` (with `teleop.joy.enabled`) |
| `params` | | extra parameter files |
| `rviz` | `false` | |

On start the backend releases Unitree's built-in sport service (`MotionSwitcher`), so gaitos
is the only one commanding the motors. It does not give it back on exit: re-enable it from the
app or the remote.

## 4. Safety

| | |
|---|---|
| Emergency stop | Unitree remote `L2+B` → `damp` |
| Remote modes | `L2+A` stand_up, `L2+X` stand_down, `L1+A` recovery_stand, `START` balance_stand, `R2+A` locomotion, `R2+X` locomotion_mpc; sticks: twist |
| Gain ramp | 1.5 s when leaving `damp` |
| Watchdog | 0.1 s: a stalled cycle damps the joints |
| Limits | joint position, speed and torque, tilt → `damp` |

Staged bring-up: (1) the DDS simulator; (2) `readonly:=true` with the robot hanging or lying:
move each leg by hand and check `/go2/joint_states`, the IMU and foot forces on `/go2/status`,
the remote; (3) hanging: `damp`, then `stand_up` with reduced gains; (4) on the ground:
standing and small body poses; (5) walking slowly.

## 5. CycloneDDS: do not subscribe to the robot from ROS 2 rolling

unitree_sdk2 carries CycloneDDS 0.10; ROS 2 rolling uses CycloneDDS 11 (or Fast DDS). A ROS 2
subscriber on an SDK topic (`ros2 topic echo /lowstate` or `/lowcmd`) makes the 0.10 side
crash: the gaitos node itself, or the robot services. Listing topics is safe. With the robot,
use a `ROS_DOMAIN_ID` other than 0: `robot.launch.py` warns when it is 0.
