//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_SYSTEM__NODEEXTENSION_HPP_
#define GAITOS_SYSTEM__NODEEXTENSION_HPP_

#include "gaitos_common/Params.hpp"
#include "gaitos_system/Gaitos.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace gaitos
{

/**
 * @class NodeExtension
 * @brief Plugin (pluginlib, base class `gaitos::NodeExtension`) that adds ROS interfaces to
 * the gaitos node: other topics and services in front of the same Gaitos (e.g. the DDS
 * interface of a robot vendor, for a stack written for the real robot to run in simulation).
 *
 * Configured in the node parameters:
 *
 *   extensions:
 *     <name>:
 *       plugin: <package>/<Class>
 *       ...                       # its own parameters (`params` of configure)
 *
 * It runs in the executor of the node (never in the control loop): it reads the state with
 * Gaitos::snapshot() and commands with the thread safe methods of Gaitos (request_mode,
 * set_twist...). Lifecycle: configure (in the node on_configure, after gaitos), activate,
 * deactivate; it is destroyed in on_cleanup.
 */
class NodeExtension
{
public:
  virtual ~NodeExtension() = default;

  /// Create the ROS interfaces. `params` is the block extensions.<name>
  virtual void configure(
    rclcpp_lifecycle::LifecycleNode & node, Gaitos & core, const Params & params) = 0;
  virtual void activate() {}
  virtual void deactivate() {}
};

}  // namespace gaitos

#endif  // GAITOS_SYSTEM__NODEEXTENSION_HPP_
