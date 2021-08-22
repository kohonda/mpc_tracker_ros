#ifndef MPC_TRACKER_Core_H
#define MPC_TRACKER_Core_H

#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include "mpc_tracker/course_manager.hpp"
#include "mpc_tracker/frenet_serret_converter.hpp"
#include "Twist.hpp"
// #include "mpc_tracker/frenet_state_filter.hpp"
#include "cgmres_solver/continuation_gmres.hpp"

/**
 * @class waypoint tracker class
 * @brief calculate control input based nonlinear MPC using C/GMRES method
 * @author Kohei Honda
 * @date Aug.2021
 */
class MPCTracker
{
public:
    /**
     * @brief default constructor
     *
     */
    MPCTracker(/* args */);

    /**
     * @brief Destroy the MPCTracker object
     *
     */
    ~MPCTracker();

private:
    /*ros system variables*/
    ros::NodeHandle nh_;           //!< @brief ros public node handle
    ros::NodeHandle private_nh_;   //!< @brief ros private node handle
    ros::Publisher pub_twist_cmd_; //!< @brief twist command topic publisher
    ros::Subscriber sub_ref_path_; //!< @brief reference path subscriber
    ros::Subscriber sub_odom_;     //!< @brief robot odom subscriber
    ros::Timer timer_control_;     //!< @brief timer for control command commutation
    std::string robot_frame_id_;
    std::string map_frame_id_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    /*control parametes*/
    double control_sampling_time_; //!< @brief control interval [s]
    // TODO: とりあえずreference speedは固定.現状どうやって車速を計画して埋め込んでいる？
    double reference_speed_; //!< @brief robot reference speed [m/s]

    /*cgmres solver parameters*/
    struct CGMRESParam
    {
        double Tf_;                        //!< @brief Length of predictive horizon in C/GMRES [s]
        double alpha_;                     //!< @brief The length horizon at time t is given by T_f * (1-exp(-alpha*t))
        int N_;                            //!< @brief The number of discrete of the predictive horizon
        double finite_distance_increment_; //!< @brief Step length of the finite difference in C/GMRES method
        double zeta_;                      //!< @brief A parameter for stabilization of the C/GMRES method.
        int kmax_;                         //!< @brief dimension of the Krylov subspace and maximum iteration number
    };
    CGMRESParam cgmres_param_;

    /*Variables*/
    struct RobotStatus
    {
        Pose robot_pose_global_;             //!< @brief Global robot pose used in MPC
        Twist robot_twist_;                  //!< @brief Robot twist used in MPC
        FrenetCoordinate robot_pose_frenet_; //!< @brief Frenet-Serret robot pose used in MPC
    };
    RobotStatus robot_status_;

    Twist prev_twist_cmd_;

    /*function used in the predictive horizon of MPC*/
    std::function<double(double)> path_curvature_ = [this](const double &x_f) { return this->course_manager_.get_curvature(x_f); };
    std::function<double(double)> trajectory_speed_ = [this](const double &x_f) { return this->course_manager_.get_speed(x_f); };
    std::function<double(double)> drivable_width_ = [this](const double &x_f) { return this->course_manager_.get_drivable_width(x_f); }; // not used now

    /*Flags*/
    bool is_robot_state_ok_ = false;
    bool initial_solution_calculate_ = false;

    /*Visualization info*/
    // TODO
    // 1. 予測状態, 2. 計算時間， 3. Fnorm

    /*Library*/
    std::unique_ptr<cgmres::ContinuationGMRES> nmpc_solver_ptr_;     //!< @brief nonlinear mpc solver pointer
    pathtrack_tools::CourseManager course_manager_;                  //!< @brief Manage reference path, reference speed and drivable area in MPC
    pathtrack_tools::FrenetSerretConverter frenet_serret_converter_; //!< @brief Converter between global coordinate and frenet-serret coordinate
    // std::unique_ptr<pathtrack_tools::FrenetStateFilter> frenet_state_filter_ptr_; //!< @brief Frenet state estimate from observed info

    /**
     * @brief calculate and publish control input
     * 
     */
    void timer_callback(const ros::TimerEvent &);

    /**
     * @brief Subscribe to odometry and update pose and twist at that time. Pose is updated from tf, and twist is updated from odometry.
     *
     * @param [in] odom
     */
    void callback_odom(const nav_msgs::Odometry &odom);

    /**
     * @brief Update reference path from higher level planner and filter it.
     * 
     * @param [in] path 
     */
    void callback_reference_path(const nav_msgs::Path &path);

    void publish_twist(const Twist &twist_cmd) const;
};

#endif