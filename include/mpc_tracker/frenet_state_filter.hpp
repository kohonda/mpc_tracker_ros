#ifndef FRENET_STATE_FILTER_H
#define FRENET_STATE_FILTER_H

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <iostream>
#include <numeric>
#include "mpc_tracker/FrenetCoordinate.hpp"
#include "mpc_tracker/Pose.hpp"
#include "mpc_tracker/Twist.hpp"

namespace pathtrack_tools {
///
/// @class FrenetStateFilter
/// @brief Estimation of state quantities of Frenet coordinate system that is impossible to observe.
///

class FrenetStateFilter {
 public:
  /**
   * @brief Default constructor
   *
   */
  FrenetStateFilter(const double& sampling_time);

  /**
   * @brief Destroy the FrenetStateFilter object
   *
   */
  ~FrenetStateFilter();
  /**
   * @brief Set the initial pose object
   *
   * @param pose_f
   */
  void set_initial_pose(const FrenetCoordinate& pose_f, const Twist& twist);

  /**
   * @brief Estimated from y_f of the past few points
   *
   * @param y_f
   * @return double dy_f
   */
  double estimate_dy_f(const double& current_y_f);

  /**
   * @brief Calculate the projection in the reference path direction
   *
   * @param current_pose_f
   * @param current_twist
   * @return double dy_f
   */
  double estimate_dy_f(const FrenetCoordinate& current_pose_f, const Twist& current_twist) const;

 private:
  const double sampling_time_;
  double prev_y_f_ = 0.0;
  const size_t dy_f_queue_size_ = 10;
  std::deque<double> dy_f_queue_;
};

}  // namespace pathtrack_tools

#endif
