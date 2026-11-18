//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include <gtest/gtest.h>

#include "gaitos_common/testing.hpp"
#include "gaitos_controllers/LegIk.hpp"
#include "gaitos_model/RobotModel.hpp"

using gaitos::RobotModel;
namespace testutil = gaitos::testutil;

static auto * const ros_env = ::testing::AddGlobalTestEnvironment(new testutil::RosEnvironment);

namespace
{
std::shared_ptr<RobotModel> nao()
{
  const std::string dir = std::string(GAITOS_SOURCE_DIR) + "/gaitos_description/robots/nao_description/";
  std::vector<rclcpp::Parameter> p{
    {"robot_description", gaitos::read_file(dir + "urdf/nao.urdf")},
    {"robot_description_semantic", gaitos::read_file(dir + "srdf/nao.srdf")},
    {"robot.base_link", "torso"},
    {"robot.legs.names", std::vector<std::string>{"left", "right"}},
    {"robot.legs.groups", std::vector<std::string>{"left_leg", "right_leg"}},
    {"robot.legs.orientation_ik", std::vector<bool>{true, true}}};
  return RobotModel::from_params(testutil::make_params({}, p));
}

double yaw(const Eigen::Isometry3d & t)
{
  return std::atan2(t.linear()(1, 0), t.linear()(0, 0));
}
}  // namespace

TEST(LegIk, CoupledLegsShareTheHipYawPitch)
{
  const auto model = nao();
  gaitos_controllers::LegIk ik;
  ik.configure(model, "stand_init");
  // Turning in place: the left foot yawed, the right one straight (NAO: one HipYawPitch motor)
  auto feet = ik.nominal_feet();
  feet[0].linear() = Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()).toRotationMatrix() * feet[0].linear();
  std::vector<double> q = ik.stance_q();
  const double error = ik.solve(feet, q);
  EXPECT_LT(error, 2e-3);
  const auto left = model->leg(0).chain.forward(q);
  const auto right = model->leg(1).chain.forward(q);
  // Mirrored HipYawPitch axes: the feet turn against each other, the body between them
  EXPECT_NEAR(yaw(left) - yaw(right), 0.15, 0.01);
  EXPECT_NEAR(yaw(left), -yaw(right), 0.02);
  EXPECT_LT((left.translation() - feet[0].translation()).norm(), 2e-3);
  EXPECT_LT((right.translation() - feet[1].translation()).norm(), 2e-3);
}
