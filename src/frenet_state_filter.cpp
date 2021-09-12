#include "mpc_tracker/frenet_state_filter.hpp"

namespace pathtrack_tools {
FrenetStateFilter::FrenetStateFilter(const double& sampling_time) : sampling_time_{sampling_time} {
  for (size_t i = 0; i < dy_f_queue_size_; i++) {
    dy_f_queue_.push_back(0.0);
  }
}

FrenetStateFilter::~FrenetStateFilter() {}

void FrenetStateFilter::set_initial_pose(const FrenetCoordinate& pose_f, const Twist& twist) {
  // 経路方向に射影して初期値は決める
  prev_y_f_ = std::sqrt(twist.x * twist.x + twist.y * twist.y) * sin(pose_f.yaw_f);
}

double FrenetStateFilter::estimate_dy_f(const double& current_y_f) {
  dy_f_queue_.pop_front();

  const double current_dy_f = (current_y_f - prev_y_f_) / sampling_time_;

  dy_f_queue_.push_back(current_dy_f);

  double averaged_dy_f = 0.0;
  if (dy_f_queue_.size() == dy_f_queue_size_) {
    averaged_dy_f =
        std::accumulate(dy_f_queue_.begin(), dy_f_queue_.end(), 0.0) / static_cast<double>(dy_f_queue_size_);
  } else {
    std::cerr << "queue size is different" << std::endl;
  }

  prev_y_f_ = current_y_f;

  return averaged_dy_f;
}

double FrenetStateFilter::estimate_dy_f(const FrenetCoordinate& pose_f, const Twist& twist) const {
  // Vel x sin(yaw_f)
  const double estimated_dy_f = std::sqrt(twist.x * twist.x + twist.y * twist.y) * sin(pose_f.yaw_f);

  return estimated_dy_f;
}

}  // namespace pathtrack_tools
