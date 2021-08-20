#include "mpc_tracker/mpc_tracker_core.hpp"

MPCTracker::MPCTracker() : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_)
{
    //  Set parameters from ros param
    /*control parameters*/
    private_nh_.param("control_sampling_time", control_sampling_time_, static_cast<double>(0.1));

    /*cgmres parameters*/
    private_nh_.param("Tf", cgmres_param_.Tf_, static_cast<double>(1.0));
    private_nh_.param("alpha", cgmres_param_.alpha_, static_cast<double>(0.5));
    private_nh_.param("N", cgmres_param_.N_, static_cast<int>(10));
    private_nh_.param("finite_distance_increment", cgmres_param_.finite_distance_increment_, static_cast<double>(0.0002));
    private_nh_.param("zeta", cgmres_param_.zeta_, static_cast<double>(62.5));
    private_nh_.param("kmax", cgmres_param_.kmax_, static_cast<int>(5));

    /*mpc parameters*/
    // TODO: set mpc parameters
    // max, min

    /*Initialize C/GMRES Solver */
    nmpc_solver_ptr_ =
        std::make_unique<cgmres::ContinuationGMRES>(cgmres_param_.Tf_, cgmres_param_.alpha_, cgmres_param_.N_, cgmres_param_.finite_distance_increment_, cgmres_param_.zeta_, cgmres_param_.kmax_);
    const double solution_initial_guess[MPC_INPUT::DIM] = { 0.01, 0.01 };
    nmpc_solver_ptr_->setParametersForInitialization(solution_initial_guess, 1e-06, 50);
    // TODO : set mpc params

    /*Initialize frenet state filter*/
    frenet_state_filter_ptr_ = std::make_unique<pathtrack_tools::FrenetStateFilter>(control_sampling_time_);

    /*set up ros system*/
    timer_control_ = nh_.createTimer(ros::Duration(control_sampling_time_), &MPCTracker::timer_callback, this);
    std::string out_twist_topic;
    std::string in_refpath_topic;
    std::string in_odom_topic;
    private_nh_.param("out_twist_topic", out_twist_topic, static_cast<std::string>("autoware_twist"));
    private_nh_.param("in_refpath_topic", in_refpath_topic, static_cast<std::string>("res_gp_path"));
    private_nh_.param("in_odom_topic", in_odom_topic, static_cast<std::string>("odom"));
    private_nh_.param("robot_frame_id", robot_frame_id_, static_cast<std::string>("base_link"));
    private_nh_.param("map_frame_id", map_frame_id_, static_cast<std::string>("map"));
    pub_twist_cmd_ = nh_.advertise<geometry_msgs::Twist>(out_twist_topic, 1);
    sub_ref_path_  = nh_.subscribe(in_refpath_topic, 1, &MPCTracker::callback_reference_path, this);
    sub_odom_      = nh_.subscribe(in_odom_topic, 1, &MPCTracker::callback_odom, this);
};

MPCTracker::~MPCTracker(){};

void MPCTracker::timer_callback(const ros::TimerEvent& te){};

void MPCTracker::callback_odom(const nav_msgs::Odometry& odom){};

void MPCTracker::callback_reference_path(const nav_msgs::Path& path){};