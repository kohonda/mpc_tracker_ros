#include "mpc_tracker/mpc_formulation.hpp"

namespace cgmres
{
  void NMPCModel::state_func(const double t, const std::vector<double> &x, const double *u, std::function<double(double)> &traj_curvature, std::function<double(double)> &traj_speed, std::function<double(double)> &drivable_width, std::vector<double> &dx) const
  {
    dx[MPC_STATE_SPACE::X_F] = u[MPC_INPUT::ANGULAR_VEL_YAW] * cos(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1);
    dx[MPC_STATE_SPACE::Y_F] = u[MPC_INPUT::ANGULAR_VEL_YAW] * sin(x[MPC_STATE_SPACE::YAW_F]);
    dx[MPC_STATE_SPACE::YAW_F] = -curvature * u[MPC_INPUT::TWIST_X] * cos(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) + u[MPC_INPUT::ANGULAR_VEL_YAW];
  }
  void NMPCModel::phixFunc(const double &t, const std::vector<double> &x, const double &curvature, const double &ref_speed, const double &drivable_width) const
  {
    phix[MPC_STATE_SPACE::X_F] = (1.0 / 2.0) * q_terminal_[MPC_STATE_SPACE::X_F] * (2 * x[MPC_STATE_SPACE::X_F] - 2 * x_ref_[MPC_STATE_SPACE::X_F]);
    phix[MPC_STATE_SPACE::Y_F] = (1.0 / 2.0) * q_terminal_[MPC_STATE_SPACE::Y_F] * (2 * x[MPC_STATE_SPACE::Y_F] - 2 * x_ref_[MPC_STATE_SPACE::Y_F]);
    phix[MPC_STATE_SPACE::YAW_F] = (1.0 / 2.0) * q_terminal_[MPC_STATE_SPACE::YAW_F] * (2 * x[MPC_STATE_SPACE::YAW_F] - 2 * x_ref_[MPC_STATE_SPACE::YAW_F]);
  }
  void NMPCModel::hxFunc(const double &t, const std::vector<double> &x, const double *u, const std::vector<double> &lmd, const double &curvature, const double &ref_speed, const double &drivable_width) const
  {
    hx[MPC_STATE_SPACE::X_F] = (1.0 / 2.0) * q_[MPC_STATE_SPACE::X_F] * (2 * x[MPC_STATE_SPACE::X_F] - 2 * x_ref_[MPC_STATE_SPACE::X_F]);
    hx[MPC_STATE_SPACE::Y_F] = -pow(curvature, 2) * lmd[MPC_STATE_SPACE::YAW_F] * u[MPC_INPUT::TWIST_X] * cos(x[MPC_STATE_SPACE::YAW_F]) / pow(-curvature * x[MPC_STATE_SPACE::Y_F] + 1, 2) + curvature * lmd[MPC_STATE_SPACE::X_F] * u[MPC_INPUT::ANGULAR_VEL_YAW] * cos(x[MPC_STATE_SPACE::YAW_F]) / pow(-curvature * x[MPC_STATE_SPACE::Y_F] + 1, 2) + (1.0 / 2.0) * q_[MPC_STATE_SPACE::Y_F] * (2 * x[MPC_STATE_SPACE::Y_F] - 2 * x_ref_[MPC_STATE_SPACE::Y_F]);
    hx[MPC_STATE_SPACE::YAW_F] = curvature * lmd[MPC_STATE_SPACE::YAW_F] * u[MPC_INPUT::TWIST_X] * sin(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) - lmd[MPC_STATE_SPACE::X_F] * u[MPC_INPUT::ANGULAR_VEL_YAW] * sin(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) + lmd[MPC_STATE_SPACE::Y_F] * u[MPC_INPUT::ANGULAR_VEL_YAW] * cos(x[MPC_STATE_SPACE::YAW_F]) + (1.0 / 2.0) * q_[MPC_STATE_SPACE::YAW_F] * (2 * x[MPC_STATE_SPACE::YAW_F] - 2 * x_ref_[MPC_STATE_SPACE::YAW_F]);
  }
  void NMPCModel::huFunc(const double &t, const std::vector<double> &x, const double *u, const std::vector<double> &lmd, const double &curvature, const double &ref_speed, const double &drivable_width) const
  {
    hu[MPC_INPUT::ANGULAR_VEL_YAW] = lmd[MPC_STATE_SPACE::X_F] * cos(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) + lmd[MPC_STATE_SPACE::YAW_F] + lmd[MPC_STATE_SPACE::Y_F] * sin(x[MPC_STATE_SPACE::YAW_F]) + 1.0 * r_[MPC_INPUT::ANGULAR_VEL_YAW] * u[MPC_INPUT::ANGULAR_VEL_YAW];
    hu[MPC_INPUT::TWIST_X] = -curvature * lmd[MPC_STATE_SPACE::YAW_F] * cos(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) + 0.5 * r_[MPC_INPUT::TWIST_X] * (-2 * ref_speed + 2 * u[MPC_INPUT::TWIST_X]);
  }

} // namespace cgmres
