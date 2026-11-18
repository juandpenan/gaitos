//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_CORE__PLUGINLOADER_HPP_
#define GAITOS_CORE__PLUGINLOADER_HPP_

#include <memory>
#include <stdexcept>
#include <string>

#include "pluginlib/class_loader.hpp"

#include "gaitos_core/PluginBase.hpp"

namespace gaitos
{

/**
 * @class PluginLoader
 * @brief pluginlib loader of one gaitos interface. Plugins are exported to gaitos_core
 * (pluginlib_export_plugin_description_file(gaitos_core plugins.xml)) and named
 * "<package>/<Class>". The loader must outlive the instances it creates.
 */
template<typename Base>
class PluginLoader
{
public:
  /// @param base_class C++ name of the interface, e.g. "gaitos::ControllerBase"
  explicit PluginLoader(const std::string & base_class)
  : loader_("gaitos_core", base_class) {}

  /**
   * @brief Create and initialize a plugin from its parameter block (`plugin: <name>` plus
   * its own parameters)
   * @param default_plugin used when the block has no `plugin`
   * @throws std::runtime_error if it cannot be created or its initialization fails
   */
  std::shared_ptr<Base> create(
    const std::string & instance_name, const Params & params, RobotModelPtr model,
    const std::string & default_plugin = "")
  {
    const auto type = params.get<std::string>("plugin", default_plugin);
    if (type.empty()) {
      throw std::runtime_error("Missing parameter " + params.full_key("plugin"));
    }
    std::shared_ptr<Base> p;
    try {
      p = loader_.createSharedInstance(type);
    } catch (const pluginlib::PluginlibException & e) {
      throw std::runtime_error("Cannot load plugin " + type + ": " + e.what());
    }
    if (!p->initialize(instance_name, params, std::move(model))) {
      throw std::runtime_error("Plugin " + instance_name + " (" + type + ") failed to initialize");
    }
    return p;
  }

  /// Plugins of this interface declared by the installed packages
  std::vector<std::string> available() const {return loader_.getDeclaredClasses();}

private:
  mutable pluginlib::ClassLoader<Base> loader_;
};

}  // namespace gaitos

#endif  // GAITOS_CORE__PLUGINLOADER_HPP_
