//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#include "gaitos_core/Gains.hpp"

#include <algorithm>

namespace gaitos
{

namespace
{
std::vector<std::size_t> resolve(const std::string & name, const RobotModel & model)
{
  if (name == "default") {
    return model.group_indices("all");
  }
  if (const auto idx = model.joint_index(name)) {
    return {*idx};
  }
  return model.group_indices(name);
}
}  // namespace

JointGains read_gains(const Params & gains, const RobotModel & model, double kp, double kd)
{
  JointGains out;
  out.kp.assign(model.joint_count(), kp);
  out.kd.assign(model.joint_count(), kd);
  // ROS parameters have no order: default, then groups, then single joints
  auto rank = [&model](const std::string & k) {
      return k == "default" ? 0 : (model.joint_index(k) ? 2 : 1);
    };
  auto keys = gains.keys();
  std::stable_sort(keys.begin(), keys.end(), [&rank](const std::string & a, const std::string & b) {
      return rank(a) < rank(b);
    });
  for (const auto & key : keys) {
    const Params entry = gains.sub(key);
    for (const auto i : resolve(key, model)) {
      out.kp[i] = entry.get<double>("kp", out.kp[i]);
      out.kd[i] = entry.get<double>("kd", out.kd[i]);
    }
  }
  return out;
}

std::vector<std::size_t> read_joint_selection(
  const Params & params, const std::string & key, const RobotModel & model,
  const std::vector<std::string> & default_value)
{
  std::vector<std::string> names;
  if (params.is_string(key)) {
    names.push_back(params.get<std::string>(key, ""));
  } else {
    names = params.get<std::vector<std::string>>(key, default_value);
  }
  std::vector<std::size_t> out;
  for (const auto & n : names) {
    for (const auto i : resolve(n == "default" ? "all" : n, model)) {
      if (std::find(out.begin(), out.end(), i) == out.end()) {
        out.push_back(i);
      }
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

}  // namespace gaitos
