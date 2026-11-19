# Operation modes

[← gaitos](../README.md#documentation)

The modes follow the Unitree documentation: the Go2 sport service
(`unitree/robot/go2/sport/sport_api.hpp`: Damp, BalanceStand, StopMove, StandUp, StandDown,
RecoveryStand, Euler, Move, Sit...) and the R1 loco FSM
(`unitree/robot/r1/loco/r1_loco_client.hpp`: ZeroTorque 0, Damp 1, Squat 2, StandUp 4,
Start 811...). These are the ones of the Go2:

| Mode | Plugin / controller | Unitree | Does |
|---|---|---|---|
| `zero_torque` | `ZeroTorqueMode` / joint | ZeroTorque | Every joint passive. |
| `damp` | `DampMode` / joint | Damp | Damping only; the safe mode. |
| `stand_up` | `PostureMode` (`stand`) / joint | StandUp | To the `stand` pose, joints locked. |
| `stand_down` | `PostureMode` (`stand_low`, `lie`) / joint | StandDown | Lies down. |
| `recovery_stand` | `PostureMode` (`lie`, `stand`) → `balance_stand` | RecoveryStand | From lying (or damped) to balanced standing. |
| `sit` | `PostureMode` (`sit`) / joint | Sit | Sits (only from standing). |
| `balance_stand` | `BalanceStandMode` / stance | BalanceStand + Euler | Body pose from `goal_pose`; a `cmd_vel` switches to `locomotion`. |
| `locomotion` | `LocomotionMode` / gait | Move | Body twist, or walk to a body relative goal. |
| `locomotion_mpc` | `LocomotionMode` / mpc | Move | The same with the convex MPC. |

The R1 has `zero_torque`, `damp`, `stand_up`, `squat`, `balance_stand`, `locomotion_lipm`
(Start + SetVelocity, where `balance_stand` goes on a twist) and `locomotion` (kinematic
gait); the NAO `stand_init`, `crouch` and `balance_stand` (NAOqi postures).

Mode rules (parameters of each mode): `allowed_from`, `next` (mode when it finishes),
`on_twist`, `controller`. The safe mode (`damp`) is entered on fall detection
(`safety.max_tilt`), on a failed mode, on a failed controller, on joint limits, on the
remote emergency stop or after `max_hardware_failures` failed reads.

## Poses in the SRDF

Named poses are `group_state` elements of the SRDF (`robot_description_semantic`). States
with the same name in different groups are merged into one whole-body pose; joints not in a
pose keep their position. The SRDF also defines the legs (one chain group per leg) and, with
its `all` group, the joint order (the motor order of the hardware).

| Robot | Poses |
|---|---|
| Go2 | `stand`, `stand_low`, `lie`, `sit`, `stretch` (stand and lie from the SDK2 `go2_stand_example`) |
| R1 | `zero`, `stand`, `squat` |
| NAO | `stand_init`, `stand_zero`, `crouch` |
