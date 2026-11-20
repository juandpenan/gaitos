//  Copyright 2026 Intelligent Robotics Lab (URJC)
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_UNITREE_SIM__DEX3HAND_HPP_
#define GAITOS_UNITREE_SIM__DEX3HAND_HPP_

#include <array>
#include <cstddef>

namespace gaitos_unitree_sim
{

/**
 * @class Dex3Hand
 * @brief Kinematic model of one Unitree Dex3-1 hand (7 motors, no physics): each motor with
 * stiffness follows its commanded position as a first-order lag, limited in speed and to its
 * joint range; a motor without stiffness (kp 0) stays where it is. The estimated torque is
 * the one of the commanded PD, limited to the motor effort. Used to emulate /dex3/<side>/state
 * for programs written for the real hands, while the simulated robot has no hands.
 */
class Dex3Hand
{
public:
  static constexpr std::size_t kMotors = 7;
  using Vector = std::array<double, kMotors>;

  Dex3Hand(const Vector & lower, const Vector & upper, double time_constant, double max_speed,
    double max_effort);

  /// Latest motor commands in motor order: position and gains (the velocity, the torque and
  /// the mode byte of HandCmd are not modelled); non-finite positions are ignored
  void command(const Vector & q, const Vector & kp, const Vector & kd);
  /// Advance `dt` seconds
  void update(double dt);

  const Vector & position() const {return q_;}
  const Vector & velocity() const {return dq_;}
  const Vector & effort() const {return tau_;}

private:
  Vector lower_;
  Vector upper_;
  double time_constant_;
  double max_speed_;
  double max_effort_;
  Vector q_{};
  Vector dq_{};
  Vector tau_{};
  Vector target_{};
  Vector kp_{};
  Vector kd_{};
};

}  // namespace gaitos_unitree_sim

#endif  // GAITOS_UNITREE_SIM__DEX3HAND_HPP_
