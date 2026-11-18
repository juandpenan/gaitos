//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CORE__SENSORBASE_HPP_
#define GAITOS_CORE__SENSORBASE_HPP_

#include "gaitos_common/RobotState.hpp"
#include "gaitos_core/PluginBase.hpp"

namespace gaitos
{

/**
 * @class ImuSensorBase
 * @brief Processing of the raw IMU of the hardware: mounting, bias, filtering, orientation
 * estimation for IMUs without it...
 */
class ImuSensorBase : public PluginBase
{
public:
  /**
   * @param raw measurement from the hardware (state.imu_raw)
   * @param dt time since the previous call [s]
   * @param[out] imu processed measurement (state.imu)
   * @return false if the measurement is not usable
   */
  virtual bool update(const ImuData & raw, double dt, ImuData & imu) = 0;
};

/**
 * @class ContactSensorBase
 * @brief Foot contact detection: from foot force sensors, from joint torques, from the
 * gait schedule...
 */
class ContactSensorBase : public PluginBase
{
public:
  /**
   * @param state robot state (joints, raw foot forces, imu)
   * @param[out] contacts contact of each foot (already sized)
   */
  virtual bool update(const RobotState & state, ContactData & contacts) = 0;
};

}  // namespace gaitos

#endif  // GAITOS_CORE__SENSORBASE_HPP_
