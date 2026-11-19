# Architecture and plugins

[← gaitos](../README.md#documentation)

```text
hardware.read -> remote/teleop -> safety -> imu -> contacts -> estimator -> mode requests
  -> mode -> controller -> gain ramp -> actuators -> hardware.write
```

| Stage | Interface (pluginlib base class) | Writes in `RobotState` |
|---|---|---|
| Hardware | `gaitos::HardwareInterface` | `joints`, `imu_raw`, `foot_force_raw`, `remote`, `external_odometry` |
| IMU | `gaitos::ImuSensorBase` | `imu` |
| Contacts | `gaitos::ContactSensorBase` | `contacts` |
| Estimator | `gaitos::StateEstimatorBase` | `base` |
| Mode | `gaitos::OperationModeBase` (through `ModeManager`) | `mode`, `reference` |
| Controller | `gaitos::ControllerBase` (the one the mode selects) | `command` |
| Actuators | `gaitos::JointActuatorBase` (one per joint group) | `motor_command` |

The cycle runs in its own thread (`real_time`: SCHED_FIFO, locked memory, CPU affinity),
or is stepped by the host (the Gazebo plugin). Subscriptions, services and the publish
timer run in the ROS executor; they talk to the cycle through a thread safe command queue
and a snapshot of the state, so the cycle never blocks on ROS. Every joint command has the
Unitree motor format, `tau = kp (q* - q) + kd (dq* - dq) + tau_ff`.

## Plugins

| Interface | Plugins |
|---|---|
| `gaitos::HardwareInterface` | `gaitos_hardware/FakeHardware`, `gaitos_mujoco/MujocoHardware`, `gaitos_unitree/UnitreeGoHardware`, `gaitos_unitree/UnitreeHgHardware` (Gazebo: provided by its system plugin) |
| `gaitos::JointActuatorBase` | `gaitos_actuators/PdPassthrough`, `gaitos_actuators/TorquePd`, `gaitos_actuators/PositionServo` |
| `gaitos::ImuSensorBase` | `gaitos_sensors/ImuPassthrough`, `gaitos_sensors/ComplementaryImu` |
| `gaitos::ContactSensorBase` | `gaitos_sensors/ForceThresholdContact`, `gaitos_sensors/TorqueContact` |
| `gaitos::StateEstimatorBase` | `gaitos_estimators/KalmanEstimator`, `gaitos_estimators/LegOdometryEstimator`, `gaitos_estimators/ExternalOdometryEstimator` |
| `gaitos::ControllerBase` | `gaitos_controllers/JointPdController`, `gaitos_controllers/StanceController`, `gaitos_controllers/GaitController`, `gaitos_dynamics/LipmZmpController`, `gaitos_dynamics/ConvexMpcController` |
| `gaitos::OperationModeBase` | `gaitos_modes/ZeroTorqueMode`, `DampMode`, `PostureMode`, `BalanceStandMode`, `LocomotionMode` |
| `gaitos::NodeExtension` (ROS interfaces of the node) | `gaitos_unitree_sim/UnitreeHgEmulator`, `gaitos_unitree_sim/UnitreeGoEmulator`, `gaitos_naoqi_sim/NaoqiDriverEmulator` |

### Writing a plugin

```cpp
#include "gaitos_core/ControllerBase.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_controllers
{
class WbcController : public gaitos::ControllerBase
{
public:
  bool update(const gaitos::RobotState & state, const gaitos::ControlReference & ref,
    double dt, gaitos::JointCommand & command) override;

protected:
  bool on_initialize() override
  {
    weight_ = params().get<double>("weight", 1.0);   // controllers.<name>.weight
    return true;
  }
  double weight_{1.0};
};
}  // namespace my_controllers

PLUGINLIB_EXPORT_CLASS(my_controllers::WbcController, gaitos::ControllerBase)
```

```xml
<!-- plugins.xml -->
<library path="my_controllers">
  <class name="my_controllers/WbcController" type="my_controllers::WbcController"
         base_class_type="gaitos::ControllerBase">
    <description>Whole body controller</description>
  </class>
</library>
```

```cmake
find_package(gaitos_core REQUIRED)
add_library(my_controllers SHARED src/WbcController.cpp)
target_link_libraries(my_controllers gaitos_core::gaitos_core)
pluginlib_export_plugin_description_file(gaitos_core plugins.xml)
```

```yaml
controllers:
  wbc: {plugin: my_controllers/WbcController, weight: 2.0}
modes:
  locomotion: {plugin: gaitos_modes/LocomotionMode, controller: wbc}
```
