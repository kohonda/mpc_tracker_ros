
#include "mpc_tracker/mpc_formulation.hpp"

namespace cgmres
{
void NMPCModel::stateFunc(const double t, const std::vector<double>& x, const double* u, std::function<double(double)>& traj_curvature, std::function<double(double)>& traj_speed,
                          std::function<double(double)>& drivable_width, std::vector<double>& dx) const
{
    const double curvature = traj_curvature(x[2]);
    // const double ref_speed = traj_speed(x[2]);

    double x0 = lf_ + lr_;
    double x1 = tan(u[0]);
    double x2 = x1 / x0;
    double x3 = x[1] + atan(lr_ * x2);
    double x4 = x[3] * cos(x3) / (-curvature * x[0] + 1);
    dx[0]     = x[3] * sin(x3);
    dx[1]     = -curvature * x4 + x2 * x[3] / sqrt(pow(lr_, 2) * pow(x1, 2) / pow(x0, 2) + 1);
    dx[2]     = x4;
    dx[3]     = u[1];
}

void NMPCModel::phixFunc(const double t, const std::vector<double>& x, std::function<double(double)>& traj_curvature, std::function<double(double)>& traj_speed,
                         std::function<double(double)>& drivable_width, std::vector<double>& phix) const
{
    // const double curvature = traj_curvature(x[2]);
    const double ref_speed = traj_speed(x[2]);

    phix[0] = (1.0 / 2.0) * q_terminal[0] * (2 * x[0] - 2 * x_ref[0]);
    phix[1] = (1.0 / 2.0) * q_terminal[1] * (2 * x[1] - 2 * x_ref[1]);
    phix[2] = (1.0 / 2.0) * q_terminal[2] * (2 * x[2] - 2 * x_ref[2]);
    phix[3] = (1.0 / 2.0) * q_terminal[3] * (2 * x[3] - 2 * ref_speed);
}

void NMPCModel::hxFunc(const double t, const std::vector<double>& x, const double* u, const std::vector<double>& lmd, std::function<double(double)>& traj_curvature,
                       std::function<double(double)>& traj_speed, std::function<double(double)>& drivable_width, std::vector<double>& hx) const
{
    const double curvature = traj_curvature(x[2]);
    const double ref_speed = traj_speed(x[2]);

    double x0  = -curvature * x[0] + 1;
    double x1  = lf_ + lr_;
    double x2  = tan(u[0]);
    double x3  = x2 / x1;
    double x4  = x[1] + atan(lr_ * x3);
    double x5  = cos(x4);
    double x6  = x5 * x[3];
    double x7  = x6 / pow(x0, 2);
    double x8  = sin(x4);
    double x9  = 1.0 / x0;
    double x10 = x8 * x9 * x[3];
    double x11 = x5 * x9;
    hx[0]      = -pow(curvature, 2) * lmd[1] * x7 + curvature * lmd[2] * x7 + (1.0 / 2.0) * q[0] * (2 * x[0] - 2 * x_ref[0]);
    hx[1]      = curvature * lmd[1] * x10 + lmd[0] * x6 - lmd[2] * x10 + (1.0 / 2.0) * q[1] * (2 * x[1] - 2 * x_ref[1]);
    hx[2]      = (1.0 / 2.0) * q[2] * (2 * x[2] - 2 * x_ref[2]);
    hx[3]      = lmd[0] * x8 + lmd[1] * (-curvature * x11 + x3 / sqrt(pow(lr_, 2) * pow(x2, 2) / pow(x1, 2) + 1)) + lmd[2] * x11 + (1.0 / 2.0) * q[3] * (2 * x[3] - 2 * ref_speed);
}

void NMPCModel::huFunc(const double t, const std::vector<double>& x, const double* u, const std::vector<double>& lmd, std::function<double(double)>& traj_curvature,
                       std::function<double(double)>& traj_speed, std::function<double(double)>& drivable_width, double* hu) const
{
    const double curvature = traj_curvature(x[2]);
    // const double ref_speed = traj_speed(x[2]);

    double x0  = tan(u[0]);
    double x1  = lf_ + lr_;
    double x2  = 1.0 / x1;
    double x3  = lr_ * x2;
    double x4  = x[1] + atan(x0 * x3);
    double x5  = pow(x0, 2);
    double x6  = pow(lr_, 2) * x5;
    double x7  = 1 + x6 / pow(x1, 2);
    double x8  = x[3] * (x5 + 1);
    double x9  = x3 * x8 / x7;
    double x10 = x9 * sin(x4) / (-curvature * x[0] + 1);
    hu[0]      = lmd[0] * x9 * cos(x4) + lmd[1] * (curvature * x10 + x2 * x8 / sqrt(x7) - 1.0 / 2.0 * x6 * x[3] * (2 * x5 + 2) / (pow(x1, 3) * pow(x7, 3.0 / 2.0))) - lmd[2] * x10 + r[0] * u[0];
    hu[1]      = lmd[3] + r[1] * u[1];
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

}  // namespace cgmres
