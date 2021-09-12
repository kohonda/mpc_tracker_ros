#include "mpc_tracker/mpc_simulator.hpp"

namespace pathtrack_tools {
MPCSimulator::MPCSimulator(const double &sampling_time) { initialize_input_queue(sampling_time); }

MPCSimulator::~MPCSimulator() {}

std::pair<Pose, Twist> MPCSimulator::update_ego_state(const double current_time, const Pose &current_ego_pose_global,
                                                      const Twist &ego_twist, const double *control_input_vec,
                                                      const double sampling_time) {
  const std::array<double, MPC_STATE_SPACE::DIM> current_ego_vehicle_state =
      ego_pose_to_state(current_ego_pose_global, ego_twist);

  auto state_func = [this](auto current_time, auto x, auto u) { return this->two_dw_with_delay(current_time, x, u); };

  std::array<double, MPC_INPUT::DIM> control_input = {control_input_vec[MPC_INPUT::ANGULAR_VEL_YAW],
                                                      control_input_vec[MPC_INPUT::ACCEL]};

  const std::array<double, MPC_STATE_SPACE::DIM> updated_robot_state =
      runge_kutta_gill(current_time, current_ego_vehicle_state, control_input, sampling_time, state_func);

  const auto [updated_robot_pose, updated_robot_twist] = ego_state_to_pose(updated_robot_state);

  return {updated_robot_pose, updated_robot_twist};
}

std::array<std::vector<double>, MPC_STATE_SPACE::DIM> MPCSimulator::reproduct_predivted_state(
    const Pose &current_ego_pose_global, const Twist &current_robot_twist,
    const std::array<std::vector<double>, MPC_INPUT::DIM> &control_input_series, const double &sampling_time) const {
  std::array<double, MPC_STATE_SPACE::DIM> current_state =
      ego_pose_to_state(current_ego_pose_global, current_robot_twist);

  std::array<std::vector<double>, MPC_STATE_SPACE::DIM> predicted_state_series;  // for return value

  auto state_func = [this](auto current_time, auto x, auto u) {
    return this->two_dw_without_delay(current_time, x, u);
  };

  for (size_t i = 0; i < control_input_series.at(0).size(); i++) {
    const std::array<double, MPC_INPUT::DIM> input = {control_input_series[MPC_INPUT::ANGULAR_VEL_YAW][i],
                                                      control_input_series[MPC_INPUT::ACCEL][i]};

    const double current_time = 0.0;

    const auto updated_state = eular(current_time, current_state, input, sampling_time, state_func);

    predicted_state_series[MPC_STATE_SPACE::X_F].push_back(updated_state[MPC_STATE_SPACE::X_F]);
    predicted_state_series[MPC_STATE_SPACE::Y_F].push_back(updated_state[MPC_STATE_SPACE::Y_F]);
    predicted_state_series[MPC_STATE_SPACE::YAW_F].push_back(updated_state[MPC_STATE_SPACE::YAW_F]);
    predicted_state_series[MPC_STATE_SPACE::TWIST_X].push_back(updated_state[MPC_STATE_SPACE::TWIST_X]);

    current_state = updated_state;
  }

  return predicted_state_series;
}

std::array<double, MPC_STATE_SPACE::DIM> MPCSimulator::ego_pose_to_state(const Pose &ego_pose,
                                                                         const Twist &ego_twist) const {
  std::array<double, MPC_STATE_SPACE::DIM> ego_state;

  ego_state.at(MPC_STATE_SPACE::X_F) = ego_pose.x;
  ego_state.at(MPC_STATE_SPACE::Y_F) = ego_pose.y;
  ego_state.at(MPC_STATE_SPACE::YAW_F) = ego_pose.yaw;
  ego_state.at(MPC_STATE_SPACE::TWIST_X) = ego_twist.x;

  return ego_state;
}

std::pair<Pose, Twist> MPCSimulator::ego_state_to_pose(
    const std::array<double, MPC_STATE_SPACE::DIM> &ego_state) const {
  Pose ego_pose;
  ego_pose.x = ego_state.at(MPC_STATE_SPACE::X_F);
  ego_pose.y = ego_state.at(MPC_STATE_SPACE::Y_F);
  ego_pose.yaw = ego_state.at(MPC_STATE_SPACE::YAW_F);

  Twist ego_twist;
  ego_twist.x = ego_state.at(MPC_STATE_SPACE::TWIST_X);
  ego_twist.y = 0.0;
  ego_twist.yaw = this->angle_input_queue_.front();

  return {ego_pose, ego_twist};
}

void MPCSimulator::initialize_input_queue(const double &sampling_time) {
  size_t accel_input_queue_size = static_cast<size_t>(round(accel_delay_time_ / sampling_time));
  for (size_t i = 0; i < accel_input_queue_size; i++) {
    accel_input_queue_.push(0.0);
  }

  size_t angle_input_queue_size = static_cast<size_t>(round(angle_delay_time_ / sampling_time));
  for (size_t i = 0; i < angle_input_queue_size; i++) {
    angle_input_queue_.push(0.0);
  }
}

double MPCSimulator::delay_accel_input(const double &raw_accel_input) {
  accel_input_queue_.push(raw_accel_input);
  const double delayed_accel_input = accel_input_queue_.front();
  accel_input_queue_.pop();

  return delayed_accel_input;
}

double MPCSimulator::delay_angle_input(const double &raw_angle_input) {
  angle_input_queue_.push(raw_angle_input);
  const double delayed_angle_input = angle_input_queue_.front();
  angle_input_queue_.pop();

  return delayed_angle_input;
}

std::array<double, MPC_STATE_SPACE::DIM> MPCSimulator::two_dw_without_delay(
    [[maybe_unused]] const double &current_time, const std::array<double, MPC_STATE_SPACE::DIM> &x,
    const std::array<double, MPC_INPUT::DIM> &u) const {
  std::array<double, MPC_STATE_SPACE::DIM> dx;
  const double curvature = 0.0;
  dx[MPC_STATE_SPACE::X_F] =
      x[MPC_STATE_SPACE::TWIST_X] * cos(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1);
  dx[MPC_STATE_SPACE::Y_F] = x[MPC_STATE_SPACE::TWIST_X] * sin(x[MPC_STATE_SPACE::YAW_F]);
  dx[MPC_STATE_SPACE::YAW_F] = -curvature * x[MPC_STATE_SPACE::TWIST_X] * cos(x[MPC_STATE_SPACE::YAW_F]) /
                                   (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) +
                               u[MPC_INPUT::ANGULAR_VEL_YAW];
  dx[MPC_STATE_SPACE::TWIST_X] = u[MPC_INPUT::ACCEL];
  return dx;
}

std::array<double, MPC_STATE_SPACE::DIM> MPCSimulator::two_dw_with_delay(
    const double &current_time, const std::array<double, MPC_STATE_SPACE::DIM> &x,
    const std::array<double, MPC_INPUT::DIM> &u) {
  const double raw_angle_input = u[MPC_INPUT::ANGULAR_VEL_YAW];
  const double raw_accel_input = u[MPC_INPUT::ACCEL];

  const double delayed_angle_input = delay_angle_input(raw_angle_input);
  const double delayed_accel_input = delay_accel_input(raw_accel_input);

  const std::array<double, MPC_INPUT::DIM> delayed_u = {delayed_angle_input, delayed_accel_input};

  const std::array<double, MPC_STATE_SPACE::DIM> dx = two_dw_without_delay(current_time, x, delayed_u);

  return dx;
}

}  // namespace pathtrack_tools
