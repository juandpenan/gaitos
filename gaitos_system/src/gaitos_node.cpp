//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

// gaitos_node: the gaitos lifecycle node. With autostart (default) it configures and
// activates itself; otherwise drive it with ros2 lifecycle / a lifecycle manager.

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "gaitos_system/GaitosNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gaitos::GaitosNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node->get_node_base_interface());
  bool autostart = true;
  if (node->has_parameter("autostart")) {
    autostart = node->get_parameter("autostart").as_bool();
  }
  if (autostart) {
    if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
      node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_FATAL(node->get_logger(), "gaitos could not start");
      rclcpp::shutdown();
      return 1;
    }
  }
  executor.spin();
  if (node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    node->deactivate();
  }
  rclcpp::shutdown();
  return 0;
}
