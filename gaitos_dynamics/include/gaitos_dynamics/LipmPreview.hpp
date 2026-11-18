//  Copyright 2026 Juan S. Cely G.
//  Licensed under the Apache License, Version 2.0 (see LICENSE)

#ifndef GAITOS_DYNAMICS__LIPMPREVIEW_HPP_
#define GAITOS_DYNAMICS__LIPMPREVIEW_HPP_

#include <Eigen/Dense>

#include <vector>

namespace gaitos_dynamics
{

/**
 * @class LipmPreview
 * @brief ZMP preview control of the linear inverted pendulum (cart-table model), after
 * Kajita et al., "Biped walking pattern generation by using preview control of zero-moment
 * point", ICRA 2003.
 *
 * Per horizontal axis, state [c, c', c''] (CoM position, velocity, acceleration), input the
 * CoM jerk, output the ZMP p = c - (z_c / g) c''. The control law
 *
 *   u(k) = -G_i sum_{i<=k} (p(i) - p_ref(i)) - G_x x(k) - sum_{j=1..N} G_p(j) p_ref(k + j)
 *
 * comes from the discrete Riccati equation of the system augmented with the integral of
 * the ZMP error. Both axes share the gains.
 */
class LipmPreview
{
public:
  /**
   * @param dt sample time [s]
   * @param com_height z_c [m]
   * @param preview_time horizon [s]
   * @param qe weight of the ZMP error
   * @param r weight of the jerk
   */
  void configure(double dt, double com_height, double preview_time, double qe = 1.0, double r = 1e-6,
    double gravity = 9.81);

  /// Number of future samples of the ZMP reference used by step()
  int preview_steps() const {return static_cast<int>(gp_.size());}
  double dt() const {return dt_;}
  double com_height() const {return zc_;}

  /**
   * @brief CoM at rest at a position
   */
  void reset(const Eigen::Vector2d & com);

  /**
   * @brief Advance one sample
   * @param zmp_ref reference from the current sample: zmp_ref[0] now, zmp_ref[j] j samples
   * ahead; at least preview_steps() + 1 elements (the last one is repeated if shorter)
   */
  void step(const std::vector<Eigen::Vector2d> & zmp_ref);

  /// Correct the CoM state with a measurement (simple observer), gain in [0, 1]
  void correct(const Eigen::Vector2d & com, const Eigen::Vector2d & com_vel, double gain_p, double gain_v);

  Eigen::Vector2d com() const {return {x_(0), y_(0)};}
  Eigen::Vector2d com_velocity() const {return {x_(1), y_(1)};}
  Eigen::Vector2d com_acceleration() const {return {x_(2), y_(2)};}
  Eigen::Vector2d zmp() const;

  const std::vector<double> & preview_gains() const {return gp_;}

private:
  double dt_{0.005};
  double zc_{0.8};
  double g_{9.81};
  Eigen::Matrix3d a_;
  Eigen::Vector3d b_;
  Eigen::RowVector3d c_;
  double gi_{0.0};
  Eigen::RowVector3d gx_;
  std::vector<double> gp_;
  Eigen::Vector3d x_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d y_{Eigen::Vector3d::Zero()};
  Eigen::Vector2d integral_{Eigen::Vector2d::Zero()};
};

}  // namespace gaitos_dynamics

#endif  // GAITOS_DYNAMICS__LIPMPREVIEW_HPP_
