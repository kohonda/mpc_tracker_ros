#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <functional>
#include <queue>
#include <utility>
#include <tuple>
#include <iostream>
#include "mpc_tracker/state_space_order.hpp"
#include "mpc_tracker/Pose.hpp"
#include "mpc_tracker/Twist.hpp"
#include "mpc_tracker/FrenetCoordinate.hpp"

// TODO : 制御入力の上下限，steerの不感帯，観測値に乗るノイズをつける．autoware.ivのsimple_plannig_simulatorが参考になる
namespace pathtrack_tools
{
///
/// @class MPCSimulator
/// @brief
///

class MPCSimulator
{
    public:
    /**
     * @brief Default constructor
     *
     */
    MPCSimulator(const double& sampling_time);

    /**
     * @brief Destroy the VechcleSimulator object
     *
     */
    ~MPCSimulator();

    /**
     * @brief Update the ego car state with applied control input by numerical integration using the fourth-order Runge-Kutta method.
     *
     * @param current_time
     * @param current_ego_pose_global
     * @param current_ego_twist
     * @param control_input_vec
     * @param sampling_time
     * @return std::tuple<Pose, Twist>
     */
    std::pair<Pose, Twist> update_ego_state(const double current_time, const Pose& current_ego_pose_global, const Twist& current_ego_twist, const double* control_input_vec,
                                            const double sampling_time);

    /**
     * @brief Update pedestrian pose and twist with constant velocity assumption
     *
     * @param current_pose
     * @param current_twist
     * @param sampling_time
     * @return std::pair<FrenetCoordinate, Twist>
     */
    std::pair<FrenetCoordinate, Twist> update_pedestrian_state(const FrenetCoordinate& current_pose, const Twist& current_twist, const double sampling_time) const;

    /**
     * @brief Reproduct predicted state in MPC from calculated control input series
     *
     * @param current_ego_pose_global
     * @param current_ego_twist
     * @param control_input_series
     * @param sampling_time
     * @return std::array<std::vector<double>, EGO_STATE_FOR_SIM::DIM>
     */
    std::array<std::vector<double>, EGO_STATE_FOR_SIM::DIM> reproduct_predivted_state(const Pose& current_ego_pose_global, const Twist& current_ego_twist,
                                                                                      const std::array<std::vector<double>, EGO_INPUT::DIM>& control_input_series, const double& sampling_time) const;

    private:
    const double vehicle_lf_      = 1.365;   // length from CG to Front axle center[m]
    const double vehicle_lr_      = 1.485;   // length from CG to Rear axle center[m]
    const double vehicle_mass_    = 1823;    // vehicle mass [kg]
    const double vehicle_inertia_ = 3315;    // vehicle moment of inertia [kgm^2]
    const double vehicle_kf_      = 51670;   // cornering stiffness of a front wheel [N/rad]
    const double vehicle_kr_      = 120500;  // cornering stiffness of a rear wheel [N/rad]

    std::queue<double> accel_input_queue_;  // buffer for accel input to describe time delay
    std::queue<double> angle_input_queue_;  // buffer for tire angle input to describe time delay
    const double accel_delay_time_ = 0.01;  // time delay for accel input [s]
    const double angle_delay_time_ = 0.01;  // time delay for tire angle input [s]

    std::array<double, EGO_STATE_FOR_SIM::DIM> ego_struct_to_state(const Pose& ego_pose, const Twist& ego_twist) const;
    std::pair<Pose, Twist> ego_state_to_struct(const std::array<double, EGO_STATE_FOR_SIM::DIM>& ego_state) const;

    // TODO : vectorには対応していないので，vectorのとき様に別の関数を用意する必要があるね
    /**
     * @brief Update the state by numerically integrating the state equation using the fourth-order Runge-Kutta method.
     *
     * @tparam StateArray
     * @tparam InputArray
     * @param current_time
     * @param current_state_array
     * @param control_input_array
     * @param sampling_time [s]
     * @param state_func
     * @return StateArray
     */
    template <typename StateArray, typename InputArray, typename StateFunc>
    StateArray runge_kutta_gill(const double current_time, const StateArray& current_state_array, const InputArray& control_input_array, const double& sampling_time, const StateFunc state_func) const
    {
        StateArray k1_vec, k2_vec, k3_vec, k4_vec, tmp_vec, updated_state_vec;

        k1_vec = state_func(current_time, current_state_array, control_input_array);
        for (size_t i = 0; i < current_state_array.size(); i++)
        {
            tmp_vec[i] = current_state_array[i] + 0.5 * sampling_time * k1_vec[i];
        }

        k2_vec = state_func(current_time + 0.5 * sampling_time, tmp_vec, control_input_array);
        for (size_t i = 0; i < current_state_array.size(); i++)
        {
            tmp_vec[i] = current_state_array[i] + sampling_time * 0.5 * (std::sqrt(2) - 1) * k1_vec[i] + sampling_time * (1 - (1 / std::sqrt(2))) * k2_vec[i];
        }

        k3_vec = state_func(current_time + 0.5 * sampling_time, tmp_vec, control_input_array);
        for (size_t i = 0; i < current_state_array.size(); i++)
        {
            tmp_vec[i] = current_state_array[i] - sampling_time * 0.5 * std::sqrt(2) * k2_vec[i] + sampling_time * (1 + (1 / std::sqrt(2))) * k3_vec[i];
        }

        k4_vec = state_func(current_time + sampling_time, tmp_vec, control_input_array);
        for (size_t i = 0; i < current_state_array.size(); i++)
        {
            updated_state_vec[i] = current_state_array[i] + (sampling_time / 6) * (k1_vec[i] + (2 - std::sqrt(2)) * k2_vec[i] + (2 + std::sqrt(2)) * k3_vec[i] + k4_vec[i]);
        }

        return updated_state_vec;
    }

    /**
     * @brief Update the state by numerically integrating the state equation using eular method
     *
     * @tparam StateArray
     * @tparam InputArray
     * @tparam StateFunc
     * @param current_time
     * @param current_state_array
     * @param control_input_array
     * @param sampling_time
     * @param state_func
     * @return StateArray
     */
    template <typename StateArray, typename InputArray, typename StateFunc>
    StateArray eular(const double current_time, const StateArray& current_state_array, const InputArray& control_input_array, const double& sampling_time, const StateFunc state_func) const
    {
        StateArray updated_state_vec, dx_vec;

        dx_vec = state_func(current_time, current_state_array, control_input_array);

        for (size_t i = 0; i < current_state_array.size(); i++)
        {
            updated_state_vec[i] = current_state_array[i] + sampling_time * dx_vec[i];
        }
        return updated_state_vec;
    }

    /**
     * @brief Pack the buffer with elements for the delay_time_.
     *
     * @param sampling_time
     */
    void initialize_input_queue(const double& sampling_time);

    double delay_accel_input(const double& raw_accel);
    double delay_angle_input(const double& raw_angle);

    /**
     * @brief State function of Kinematic Bicycle Model without input delay
     *
     * @param current time[s]
     * @param x=[y_g, yaw_g, x_g, vel]
     * @param u=[tire_angle, accel]
     * @return dx = d/dt[y_g, yaw_g, x_g, vel]
     */
    std::array<double, EGO_STATE_FOR_SIM::DIM> fx_kbm_without_delay(const double current_time, const std::array<double, EGO_STATE_FOR_SIM::DIM>& x, const double* u) const;

    /**
     * @brief State function of Kinematic Bicycle Model with input delay
     *
     * @param current time[s]
     * @param x=[y_g, yaw_g, x_g, vel]
     * @param u=[tire_angle, accel]
     * @return dx = d/dt[y_g, yaw_g, x_g, vel]
     */
    std::array<double, EGO_STATE_FOR_SIM::DIM> fx_kbm_with_delay(const double current_time, const std::array<double, EGO_STATE_FOR_SIM::DIM>& x, const double* u);

    /**
     * @brief State function of Dynamic Bicycle Model without input delay
     *
     * @param current_time [s]
     * @param x = [y_g, dot_y_g, theta_g, dot_theta_g, x_g, vel ]
     * @param u = [tire_angle, accel]
     * @return dx = d/dt[y_g, dot_y_g, theta_g, dot_theta_g, x_g, vel ]
     */
    std::array<double, EGO_STATE_FOR_SIM::DIM> fx_dbm_without_delay(const double current_time, const std::array<double, EGO_STATE_FOR_SIM::DIM>& x, const double* u) const;

    /**
     * @brief
     *
     * @param current_pose_g
     * @param current_twist
     * @param u
     * @return dx = d/dt[x_g, y_g, yaw_g, twist.x, twist.y, twist.yaw]
     */
    std::array<double, EGO_STATE_FOR_SIM::DIM> fx_dbm_without_delay(const Pose& current_pose_g, const Twist& current_twist, const double* u) const;

    /**
     * @brief State function of Dynamic Bicycle Model with input delay
     *
     * @param current_time [s]
     * @param x = [y_g, dot_y_g, theta_g, dot_theta_g, x_g, vel ]
     * @param u = [tire_angle, accel]
     * @return dx = d/dt[y_g, dot_y_g, theta_g, dot_theta_g, x_g, vel ]
     */
    std::array<double, EGO_STATE_FOR_SIM::DIM> fx_dbm_with_delay(const double current_time, const std::array<double, EGO_STATE_FOR_SIM::DIM>& x, const double* u);

    /**
     * @brief State function of Crossing Pedestrian
     *
     * @param current_time
     * @param x
     * @return std::array<double, PEDESTRIAN_STATE::DIM>
     */
    std::array<double, PEDESTRIAN_STATE::DIM> fx_crossing_pedestrian(const double current_time, const std::array<double, PEDESTRIAN_STATE::DIM>& x) const;

    /**
     * @brief State function of Dynamic Bicycle Model improved inLow-speed range without input delay
     *
     * @param current_time
     * @param x
     * @param u
     * @return std::array<double, EGO_STATE_FOR_SIM::DIM>
     */
    std::array<double, EGO_STATE_FOR_SIM::DIM> fx_dbml_without_delay(const double current_time, const std::array<double, EGO_STATE_FOR_SIM::DIM>& x, const double* u) const;
};

}  // namespace pathtrack_tools
