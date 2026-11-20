# Safety and the real robot

[← gaitos](../README.md#documentation)

| Protection | Parameters | Does |
|---|---|---|
| Read only | `dry_run` (`readonly:=true`) | Everything runs but the hardware is never written, not even on exit or by the watchdog. |
| Joint limits | `safety.limits` | Position beyond the URDF limits + margin, speed or measured torque over a scale of the limits → safe mode. |
| Gain ramp | `safety.gain_ramp_time` | Leaving `damp` / `zero_torque`, kp and feedforward rise from 0. |
| Watchdog | `safety.watchdog_timeout` | If the cycle stalls, another thread calls the hardware `emergency_stop()` (damped joints). |
| Remote | `teleop` | Unitree wireless controller (`LowState.wireless_remote`) or `sensor_msgs/Joy` (`teleop.joy`): emergency stop `L2+B`, modes on button combinations, twist from the sticks. |
| Falls | `safety.max_tilt` | Tilt over the limit → safe mode. |
| External joint commands | `external_joints.*` | `set_joint_commands` (vendor emulators): non-finite commands ignored; position, speed and torque within the URDF limits; gains ≥ 0 with no upper limit; back to the controller over `release_time` after `command_timeout` or a mode change, at once in a passive mode. |

The Unitree parameter files enable the ramp (1.5 s), the watchdog (0.1 s) and the remote
(Go2: `L2+A` stand_up, `L2+X` stand_down, `L1+A` recovery_stand, `START` balance_stand,
`R2+A` locomotion, `R2+X` locomotion_mpc; R1: `L2+A` stand_up, `L2+X` squat, `START`
balance_stand, `R2+A` locomotion_lipm).

Suggested sequence: (1) the DDS simulator; (2) read only with the robot hanging or lying:
check joints and signs (`/go2/joint_states`), IMU, foot forces and remote
(`/go2/status`); (3) hanging, passive modes and `stand_up` with reduced gains; (4) on the
ground, standing and small pose goals; (5) walking slowly; the R1 with a harness. The R1
LIPM gains (kp 600) come from simulation: check them against the motors.
