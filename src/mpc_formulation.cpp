
#include "mpc_tracker/mpc_formulation.hpp"

namespace cgmres
{
    void NMPCModel::stateFunc([[maybe_unused]] const double t, const std::vector<double> &x, const double *u, std::function<double(double)> &traj_curvature, [[maybe_unused]] std::function<double(double)> &traj_speed,
                              [[maybe_unused]] std::function<double(double)> &drivable_width, std::vector<double> &dx) const
    {
        const double curvature = traj_curvature(x[MPC_STATE_SPACE::X_F]);

        dx[MPC_STATE_SPACE::X_F] = x[MPC_STATE_SPACE::TWIST_X] * cos(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1);
        dx[MPC_STATE_SPACE::Y_F] = x[MPC_STATE_SPACE::TWIST_X] * sin(x[MPC_STATE_SPACE::YAW_F]);
        dx[MPC_STATE_SPACE::YAW_F] = -curvature * x[MPC_STATE_SPACE::TWIST_X] * cos(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) + u[MPC_INPUT::ANGULAR_VEL_YAW];
        dx[MPC_STATE_SPACE::TWIST_X] = u[MPC_INPUT::ACCEL];
    }

    void NMPCModel::phixFunc([[maybe_unused]] const double t, const std::vector<double> &x, [[maybe_unused]] std::function<double(double)> &traj_curvature, std::function<double(double)> &traj_speed,
                             [[maybe_unused]] std::function<double(double)> &drivable_width, std::vector<double> &phix) const
    {
        // const double curvature = traj_curvature(x[MPC_STATE_SPACE::X_F]);
        const double ref_speed = traj_speed(x[MPC_STATE_SPACE::X_F]);

        phix[MPC_STATE_SPACE::X_F] = (1.0 / 2.0) * q_terminal_[MPC_STATE_SPACE::X_F] * (2 * x[MPC_STATE_SPACE::X_F] - 2 * x_ref_[MPC_STATE_SPACE::X_F]);
        phix[MPC_STATE_SPACE::Y_F] = (1.0 / 2.0) * q_terminal_[MPC_STATE_SPACE::Y_F] * (2 * x[MPC_STATE_SPACE::Y_F] - 2 * x_ref_[MPC_STATE_SPACE::Y_F]);
        phix[MPC_STATE_SPACE::YAW_F] = (1.0 / 2.0) * q_terminal_[MPC_STATE_SPACE::YAW_F] * (2 * x[MPC_STATE_SPACE::YAW_F] - 2 * x_ref_[MPC_STATE_SPACE::YAW_F]);
        phix[MPC_STATE_SPACE::TWIST_X] = (1.0 / 2.0) * q_terminal_[MPC_STATE_SPACE::TWIST_X] * (-2 * ref_speed + 2 * x[MPC_STATE_SPACE::TWIST_X]);
    }

    void NMPCModel::hxFunc([[maybe_unused]] const double t, const std::vector<double> &x, [[maybe_unused]] const double *u, const std::vector<double> &lmd, std::function<double(double)> &traj_curvature,
                           std::function<double(double)> &traj_speed, [[maybe_unused]] std::function<double(double)> &drivable_width, std::vector<double> &hx) const
    {
        const double curvature = traj_curvature(x[MPC_STATE_SPACE::X_F]);
        const double ref_speed = traj_speed(x[MPC_STATE_SPACE::X_F]);

        hx[MPC_STATE_SPACE::X_F] = (1.0 / 2.0) * q_[MPC_STATE_SPACE::X_F] * (2 * x[MPC_STATE_SPACE::X_F] - 2 * x_ref_[MPC_STATE_SPACE::X_F]);
        hx[MPC_STATE_SPACE::Y_F] = -pow(curvature, 2) * lmd[MPC_STATE_SPACE::YAW_F] * x[MPC_STATE_SPACE::TWIST_X] * cos(x[MPC_STATE_SPACE::YAW_F]) / pow(-curvature * x[MPC_STATE_SPACE::Y_F] + 1, 2) + curvature * lmd[MPC_STATE_SPACE::X_F] * x[MPC_STATE_SPACE::TWIST_X] * cos(x[MPC_STATE_SPACE::YAW_F]) / pow(-curvature * x[MPC_STATE_SPACE::Y_F] + 1, 2) + (1.0 / 2.0) * q_[MPC_STATE_SPACE::Y_F] * (2 * x[MPC_STATE_SPACE::Y_F] - 2 * x_ref_[MPC_STATE_SPACE::Y_F]);
        hx[MPC_STATE_SPACE::YAW_F] = curvature * lmd[MPC_STATE_SPACE::YAW_F] * x[MPC_STATE_SPACE::TWIST_X] * sin(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) - lmd[MPC_STATE_SPACE::X_F] * x[MPC_STATE_SPACE::TWIST_X] * sin(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) + lmd[MPC_STATE_SPACE::Y_F] * x[MPC_STATE_SPACE::TWIST_X] * cos(x[MPC_STATE_SPACE::YAW_F]) + (1.0 / 2.0) * q_[MPC_STATE_SPACE::YAW_F] * (2 * x[MPC_STATE_SPACE::YAW_F] - 2 * x_ref_[MPC_STATE_SPACE::YAW_F]);
        hx[MPC_STATE_SPACE::TWIST_X] = -curvature * lmd[MPC_STATE_SPACE::YAW_F] * cos(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) + lmd[MPC_STATE_SPACE::X_F] * cos(x[MPC_STATE_SPACE::YAW_F]) / (-curvature * x[MPC_STATE_SPACE::Y_F] + 1) + lmd[MPC_STATE_SPACE::Y_F] * sin(x[MPC_STATE_SPACE::YAW_F]) + (1.0 / 2.0) * q_[MPC_STATE_SPACE::TWIST_X] * (-2 * ref_speed + 2 * x[MPC_STATE_SPACE::TWIST_X]);
    }

    void NMPCModel::huFunc([[maybe_unused]] const double t, [[maybe_unused]] const std::vector<double> &x, const double *u, const std::vector<double> &lmd, [[maybe_unused]] std::function<double(double)> &traj_curvature,
                           [[maybe_unused]] std::function<double(double)> &traj_speed, [[maybe_unused]] std::function<double(double)> &drivable_width, double *hu) const
    {
        // const double curvature = traj_curvature(x[MPC_STATE_SPACE::X_F]);
        // const double ref_speed = traj_speed(x[MPC_STATE_SPACE::X_F]);

        hu[MPC_INPUT::ANGULAR_VEL_YAW] = lmd[MPC_STATE_SPACE::YAW_F] + 1.0 * r_[MPC_INPUT::ANGULAR_VEL_YAW] * u[MPC_INPUT::ANGULAR_VEL_YAW];
        hu[MPC_INPUT::ACCEL] = lmd[MPC_STATE_SPACE::TWIST_X] + 1.0 * r_[MPC_INPUT::ACCEL] * u[MPC_INPUT::ACCEL] + ((u[MPC_INPUT::ACCEL] - 0.29999999999999999 < -1.0) ? (rho_g_ / (0.29999999999999999 - u[MPC_INPUT::ACCEL])) : ((1.0 / 2.0) * rho_g_ * (2.0 * u[MPC_INPUT::ACCEL] + 3.3999999999999999))) + ((u[MPC_INPUT::ACCEL] + 0.29999999999999999 > 1.0) ? (-rho_g_ / (u[MPC_INPUT::ACCEL] + 0.29999999999999999)) : ((1.0 / 2.0) * rho_g_ * (2.0 * u[MPC_INPUT::ACCEL] - 3.3999999999999999)));
    }

    int NMPCModel::dim_state() const
    {
        return dim_state_;
    }

    int NMPCModel::dim_control_input() const
    {
        return dim_control_input_;
    }

    int NMPCModel::dim_constraints() const
    {
        return dim_constraints_;
    }

    void NMPCModel::set_parameters(const std::array<double, MPC_STATE_SPACE::DIM> &q, const std::array<double, MPC_STATE_SPACE::DIM> &q_terminal, const std::array<double, MPC_INPUT::DIM> &r)
    {
        q_ = q;
        q_terminal_ = q_terminal;
        r_ = r;
    }

} // namespace cgmres
