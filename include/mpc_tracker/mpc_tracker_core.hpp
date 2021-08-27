#ifndef MPC_TRACKER_Core_H
#define MPC_TRACKER_Core_H

#include <memory>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "mpc_tracker/course_manager.hpp"
#include "mpc_tracker/frenet_serret_converter.hpp"
#include "Twist.hpp"
// #include "mpc_tracker/frenet_state_filter.hpp"
#include "cgmres_solver/continuation_gmres.hpp"
#include "mpc_tracker/mpc_simulator.hpp"
#include "StopWatch.hpp"

#include "autoware_msgs/Trigger.h"

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
    ros::ServiceServer service_control_trigger_; //!< @brief control trigger from higher level planner
    ros::ServiceClient client_finish_goal_;      //!< @brief client for announce of finishing drive

    /*control system parametes*/
    double control_sampling_time_; //!< @brief control interval [s]
    // TODO: とりあえずreference speedは固定.-> reference pathをposeのみならず車速埋め込みにする
    double reference_speed_;            //!< @brief robot reference speed [m/s]
    int zero_speed_points_around_goal_; //!< @brief Number of waypoints which are set the speed to zero for stop at the goal
    double goal_judgment_distance_;     //!< @brief Threshold for goal judgment [m]

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

    /*MPC parameters*/
    struct MPCParam
    {
        std::array<double, MPC_STATE_SPACE::DIM> q_;          //!< @brief Weight of state for stage cost in MPC
        std::array<double, MPC_STATE_SPACE::DIM> q_terminal_; //!< @brief Weight of state for terminal cost in MPC
        std::array<double, MPC_INPUT::DIM> r_;                //!< @brief Weight of input for stage cost in MPC
    };
    MPCParam mpc_param_;

    /*Variables*/
    struct RobotStatus
    {
        Pose robot_pose_global_;             //!< @brief Global robot pose used in MPC
        Twist robot_twist_;                  //!< @brief Robot twist used in MPC
        FrenetCoordinate robot_pose_frenet_; //!< @brief Frenet-Serret robot pose used in MPC
    };
    RobotStatus robot_status_;

    Twist prev_twist_cmd_;          //!< @brief published twist command just before
    Pose reference_path_goal_pose_; //!< @brief pose at the final reference point

    /*function used in the predictive horizon of MPC*/
    std::function<double(double)> path_curvature_ = [this](const double &x_f) { return this->course_manager_.get_curvature(x_f); };      //!< @brief return curvature from pose x_f in frenet coordinate
    std::function<double(double)> trajectory_speed_ = [this](const double &x_f) { return this->course_manager_.get_speed(x_f); };        //!< @brief return reference speed from pose x_f in frenet coordinate
    std::function<double(double)> drivable_width_ = [this](const double &x_f) { return this->course_manager_.get_drivable_width(x_f); }; // not used now

    /*Flags*/
    bool is_robot_state_ok_ = false;          //!< @brief Check getting robot observed info
    bool initial_solution_calculate_ = false; //!< @brief Initialize C/GMRES method using Newton-method
    bool is_finish_goal_ = false;             //!< @brief Check reached the goal
    bool is_permit_control_ = false;          //!< @brief Check control permission from higher level planner

    /*Visualization info*/
    ros::Publisher pub_predictive_pose_;  //! @brief MPC predictive pose publisher
    ros::Publisher pub_calculation_time_; //! @brief calculation time publisher
    ros::Publisher pub_F_norm_;           //! @brief F norm is Deviation from optimal solution

    /*Library*/
    std::unique_ptr<cgmres::ContinuationGMRES> nmpc_solver_ptr_;       //!< @brief nonlinear mpc solver pointer
    pathtrack_tools::CourseManager course_manager_;                    //!< @brief Manage reference path, reference speed and drivable area in MPC
    pathtrack_tools::FrenetSerretConverter frenet_serret_converter_;   //!< @brief Converter between global coordinate and frenet-serret coordinate
    std::unique_ptr<pathtrack_tools::MPCSimulator> mpc_simulator_ptr_; //!< @brief Reproduct MPC predictive state
    StopWatch stop_watch_;
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

    /**
     * @brief Receive control permission from higher level planners
     *
     * @param request
     * @param response
     */
    bool callback_control_trigger(autoware_msgs::Trigger::Request &request, autoware_msgs::Trigger::Response &response);

    /**
     * @brief Publish geometry_msgs::Twist
     *
     * @param twist_cmd
     */
    void publish_twist(const Twist &twist_cmd) const;

    /**
     * @brief converter form predictive state in MPC to MarkerArray for rviz
     *
     * @param predictive_state
     * @param name_space
     * @param frame_id
     * @param r
     * @param g
     * @param b
     * @param z
     * @return visualization_msgs::MarkerArray
     */
    visualization_msgs::MarkerArray convert_predictivestate2marker(const std::array<std::vector<double>, MPC_STATE_SPACE::DIM> &predictive_state, const std::string &name_space,
                                                                   const std::string &frame_id, const double &r, const double &g, const double &b, const double &z) const;

    /**
     * @brief report reaching goal to higher level planner
     *
     */
    void report_reached_goal();
};

#endif