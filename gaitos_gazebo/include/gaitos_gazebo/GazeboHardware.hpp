//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_GAZEBO__GAZEBOHARDWARE_HPP_
#define GAITOS_GAZEBO__GAZEBOHARDWARE_HPP_

#include <vector>

#include "gaitos_core/HardwareInterface.hpp"

namespace gaitos_gazebo
{

/**
 * @class GazeboHardware
 * @brief Backend fed by the GaitosGzSystem plugin: the plugin copies the simulated
 * sensors into it before each gaitos cycle and applies the last joint command (PD closed
 * at every physics step) after it. Time is the simulation time.
 */
class GazeboHardware : public gaitos::HardwareInterface
{
public:
  struct Input
  {
    double time{0.0};
    gaitos::JointState joints;
    gaitos::ImuData imu;
    std::vector<double> foot_force;
    gaitos::Odometry ground_truth;
    bool has_ground_truth{false};
  };

  void set_input(const Input & input) {input_ = input;}
  Input & input() {return input_;}
  const gaitos::JointCommand & command() const {return command_;}

  bool read(gaitos::RobotState & state) override;
  bool write(const gaitos::JointCommand & command) override;
  double now() const override {return input_.time;}
  bool is_simulation() const override {return true;}

protected:
  bool on_initialize() override;

private:
  Input input_;
  gaitos::JointCommand command_;
  uint64_t seq_{0};
};

}  // namespace gaitos_gazebo

#endif  // GAITOS_GAZEBO__GAZEBOHARDWARE_HPP_
