//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CORE__PLUGINBASE_HPP_
#define GAITOS_CORE__PLUGINBASE_HPP_

#include <memory>
#include <stdexcept>
#include <string>

#include "gaitos_common/Logger.hpp"
#include "gaitos_common/Params.hpp"
#include "gaitos_model/RobotModel.hpp"

namespace gaitos
{

/**
 * @class PluginBase
 * @brief Common part of every gaitos plugin (hardware, actuators, sensors, estimators,
 * controllers and operation modes).
 *
 * A plugin is created with pluginlib from the `plugin` parameter of its block of the ROS 2
 * parameters (e.g. `controllers.gait.plugin`) and reads its own parameters from the same
 * block through params(). Every method is called from the
 * gaitos cycle thread, which may run with real-time priority: avoid blocking calls and
 * memory allocations in the update methods.
 */
class PluginBase
{
public:
  virtual ~PluginBase() = default;

  /**
   * @brief Store the configuration and configure the plugin
   * @param name instance name (configuration key), used for logging
   * @param params configuration block of the instance
   * @param model robot description
   * @return result of on_initialize()
   */
  bool initialize(const std::string & name, const Params & params, RobotModelPtr model)
  {
    name_ = name;
    params_ = params;
    model_ = std::move(model);
    logger_ = Logger("gaitos." + name);
    return on_initialize();
  }

  /**
   * @brief Reset the internal state (filters, integrators, phases)
   */
  virtual void reset() {}

  const std::string & name() const {return name_;}

protected:
  /**
   * @brief Read parameters and allocate. Called once by initialize().
   * @return false if the plugin cannot work with this configuration
   */
  virtual bool on_initialize() = 0;

  const Params & params() const {return params_;}

  const RobotModel & model() const
  {
    if (!model_) {
      throw std::runtime_error("gaitos plugin " + name_ + " used without a robot model");
    }
    return *model_;
  }

  RobotModelPtr model_ptr() const {return model_;}
  const Logger & logger() const {return logger_;}

private:
  std::string name_;
  Params params_;
  RobotModelPtr model_;
  Logger logger_;
};

}  // namespace gaitos

#endif  // GAITOS_CORE__PLUGINBASE_HPP_
