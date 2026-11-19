# Controllers and state estimation

[← gaitos](../README.md#documentation)

The controllers are `gaitos::ControllerBase` plugins and the estimators `gaitos::StateEstimatorBase` plugins (see [Architecture and plugins](architecture.md)); a mode selects its controller (`controller: <name>`, see [Operation modes](modes.md)).

## Kinematic controllers (`gaitos_controllers`)

- **`StanceController`**: all feet on the ground at the nominal stance of an SRDF pose; the
  body moves over them following `body_offset` and `body_rpy` (limited in value and rate),
  solved with inverse kinematics (6D for legs with `orientation_ik`).
- **`GaitController`**: periodic gait for any number of legs (`gaits.<name>.offsets`,
  `period`, `duty`, `step_height`): stance feet move against the body twist, swing feet
  follow a cycloid to the Raibert touchdown point; a PI loop on the estimated body velocity
  (`velocity_feedback`), attitude feedback and lateral sway (bipeds).

## Dynamic controllers (`gaitos_dynamics`)

- **`LipmZmpController`** (bipeds), after Kajita et al., ICRA 2003: footstep planner from
  the body twist; ZMP reference from sole to sole; preview control of the cart-table model
  (discrete Riccati equation) for the CoM; cycloid swing foot; 6D inverse kinematics of the
  legs. Feedforward of the LIPM ground reaction at the center of pressure
  (τ = −Jᵀ [F; M], `feedforward`); DCM stabilizer moving the center of pressure with the
  error of the measured DCM (`dcm_feedback`, ankle strategy); heading recovery
  (`heading_gain`, `heading_body_gain`); attitude feedback.
- **`ConvexMpcController`** (quadrupeds), after Di Carlo et al., IROS 2018: single rigid body
  with the composite inertia of the URDF, linearized around the yaw, condensed QP on the
  ground reaction forces with friction pyramids and normal force bounds, solved with OSQP
  (`osqp_vendor`, warm started); stance legs apply τ = −Jᵀ Rᵀ f, swing legs follow the
  Raibert footstep with joint PD. To stop, the gait goes on until every foot is in stance and
  on the ground (measured contact), and then all the legs stop together; stopped, the legs
  hold that posture with `gains.standing` under the MPC forces.

## State estimation

`KalmanEstimator`: base position and velocity plus every foot (world frame); IMU prediction;
corrections with leg kinematics, zero velocity of the stance feet and their height
(`ground_height`); orientation from the IMU, yaw corrected by external sources.

```yaml
external_odometry: {sources: [mocap], topics: [/mocap/odom]}   # nav_msgs/Odometry topics
estimator:
  plugin: gaitos_estimators/KalmanEstimator
  external:
    mocap: {position: true, yaw: true, position_noise: 0.01, align: true, gate: 11.3}
```

Sources: topics as above, `ground_truth` (simulators), `unitree_sport` (Go2 sport odometry).
