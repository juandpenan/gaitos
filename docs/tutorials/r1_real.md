# R1: the real robot

[← Tutorials](README.md)

The real Unitree R1 through `unitree_sdk2` (`unitree_hg`: `rt/lowstate`, `rt/lowcmd`), from the
`gaitos_unitree/UnitreeHgHardware` plugin. **Not tested on the real robot**: use a harness, and
the read only mode first. To drive the R1 with Unitree's own controller instead, use the
`unitree_r1` stack (and gaitos only in simulation, [R1 in MuJoCo](r1_mujoco.md#5-unitree_r1-against-this-simulation)).

## 1. Before starting

```bash
cd ~/ws/legged_ws
source install/setup.bash
export ROS_DOMAIN_ID=10      # NOT 0 with the robot connected
```

## 2. Launch

```bash
ros2 launch gaitos_unitree robot.launch.py robot:=r1 readonly:=true network_interface:=eth0   # 1) read only
ros2 launch gaitos_unitree robot.launch.py robot:=r1 network_interface:=eth0                  # 2) commanding
```

The arguments are those of [Go2: the real robot](go2_real.md#3-the-robot). The motor slots follow
the documented order of the R1 (0–11 legs, 12–13 waist, 15–19 and 22–26 arms, 29–30 head). On
start the backend releases Unitree's loco service; it does not give it back on exit.

## 3. Safety

| | |
|---|---|
| Emergency stop | Unitree remote `L2+B` → `damp` |
| Remote modes | `L2+A` stand_up, `L2+X` squat, `START` balance_stand, `R2+A` locomotion_lipm; sticks: twist |
| Gain ramp, watchdog, limits | 1.5 s, 0.1 s, joint limits and tilt → `damp` |

Staged bring-up: (1) `readonly:=true` hanging: check joints and signs (`/r1/joint_states`), IMU,
remote (`/r1/status`); (2) hanging: `damp`, `stand_up` with reduced gains; (3) on the ground with
the harness: standing, small body poses; (4) walking slowly. The LIPM gains (legs kp 600) come
from simulation: check them against the motors.

## 4. With unitree_r1 on the same network

- Do not run gaitos and Unitree's locomotion at the same time: gaitos releases the loco service
  and writes `rt/lowcmd`; `r1_locomotion`, `r1_gesture`, `r1_arm` (`arm_sdk`) stop working.
- CycloneDDS: a ROS 2 rolling subscriber on an SDK topic makes the CycloneDDS 0.10 side crash
  (gaitos itself or the robot services); use a `ROS_DOMAIN_ID` other than 0.
