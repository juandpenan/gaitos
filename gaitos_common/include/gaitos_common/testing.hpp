//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

// Test helpers (gtest only): ROS 2 initialization and nodes loaded with parameter files.

#ifndef GAITOS_COMMON__TESTING_HPP_
#define GAITOS_COMMON__TESTING_HPP_

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "gaitos_common/Params.hpp"

namespace gaitos
{
namespace testutil
{

/// The nodes behind make_params(): they live until the end of the test program
inline std::vector<rclcpp::Node::SharedPtr> & kept_nodes()
{
  static std::vector<rclcpp::Node::SharedPtr> nodes;
  return nodes;
}

/// rclcpp::init / shutdown around the whole test program. The kept nodes are destroyed before
/// the shutdown: destroying them later, during static destruction, can hang with CycloneDDS.
class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override
  {
    kept_nodes().clear();
    rclcpp::shutdown();
  }
};

/// A node with the parameters of the files (in order) plus overrides
inline rclcpp::Node::SharedPtr make_node(
  const std::vector<std::string> & files, const std::vector<rclcpp::Parameter> & overrides = {})
{
  static std::atomic<int> count{0};
  std::vector<std::string> args{"--ros-args"};
  for (const auto & f : files) {
    args.push_back("--params-file");
    args.push_back(f);
  }
  return std::make_shared<rclcpp::Node>(
    "gaitos_test_" + std::to_string(count++),
    rclcpp::NodeOptions().arguments(args).parameter_overrides(overrides)
    .automatically_declare_parameters_from_overrides(true));
}

inline Params make_params(
  const std::vector<std::string> & files, const std::vector<rclcpp::Parameter> & overrides = {})
{
  auto & keep = kept_nodes();
  keep.push_back(make_node(files, overrides));
  return Params(keep.back()->get_node_parameters_interface());
}

}  // namespace testutil
}  // namespace gaitos

#endif  // GAITOS_COMMON__TESTING_HPP_
