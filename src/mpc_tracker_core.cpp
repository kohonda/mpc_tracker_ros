#include "mpc_tracker/mpc_tracker_core.hpp"

MPCTracker::MPCTracker() : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_), prev_twist_cmd_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
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
    private_nh_.param("reference_speed", reference_speed_, static_cast<double>(1.0));
    // max, min

    /*Initialize C/GMRES Solver */
    nmpc_solver_ptr_ =
        std::make_unique<cgmres::ContinuationGMRES>(cgmres_param_.Tf_, cgmres_param_.alpha_, cgmres_param_.N_, cgmres_param_.finite_distance_increment_, cgmres_param_.zeta_, cgmres_param_.kmax_);
    const double solution_initial_guess[MPC_INPUT::DIM] = {0.01, 0.01};
    nmpc_solver_ptr_->setParametersForInitialization(solution_initial_guess, 1e-06, 50);
    // TODO : set mpc params

    /*Initialize frenet state filter*/
    // frenet_state_filter_ptr_ = std::make_unique<pathtrack_tools::FrenetStateFilter>(control_sampling_time_);

    /*set up ros system*/
    timer_control_ = nh_.createTimer(ros::Duration(control_sampling_time_), &MPCTracker::timer_callback, this);
    std::string out_twist_topic;
    std::string in_refpath_topic;
    std::string in_odom_topic;
    private_nh_.param("out_twist_topic", out_twist_topic, static_cast<std::string>("twist_cmd"));
    private_nh_.param("in_refpath_topic", in_refpath_topic, static_cast<std::string>("reference_path"));
    private_nh_.param("in_odom_topic", in_odom_topic, static_cast<std::string>("odom"));
    private_nh_.param("robot_frame_id", robot_frame_id_, static_cast<std::string>("base_link"));
    private_nh_.param("map_frame_id", map_frame_id_, static_cast<std::string>("map"));
    pub_twist_cmd_ = nh_.advertise<geometry_msgs::Twist>(out_twist_topic, 1); // TODO: twist_timestampedに変更
    sub_ref_path_ = nh_.subscribe(in_refpath_topic, 1, &MPCTracker::callback_reference_path, this);
    sub_odom_ = nh_.subscribe(in_odom_topic, 1, &MPCTracker::callback_odom, this);
};

MPCTracker::~MPCTracker(){};

void MPCTracker::timer_callback([[maybe_unused]] const ros::TimerEvent &te)
{
    /*status check*/
    const int path_size = course_manager_.get_path_size();
    if (path_size == 0 || !is_robot_state_ok_)
    {
        ROS_INFO("[MPC] MPC is not solved. ref_path_size: %d, is_pose_set: %d", path_size, is_robot_state_ok_);
        publish_twist(prev_twist_cmd_);
        return;
    }

    /*coordinate convert from euclid to frenet-serret*/
    robot_status_.robot_pose_frenet_ = frenet_serret_converter_.global2frenet(course_manager_.get_mpc_course(), robot_status_.robot_pose_global_);

    /*Set state vector*/
    const std::vector<double> state_vec{robot_status_.robot_pose_frenet_.x_f, robot_status_.robot_pose_frenet_.y_f, robot_status_.robot_pose_frenet_.yaw_f};

    /*Control input*/
    double control_input_vec[MPC_INPUT::DIM] = {0.0};

    /*Control input series for visualization */
    std::array<std::vector<double>, MPC_INPUT::DIM> control_input_series; // TODO: デバッグ用にrvizに予測経路として表示する

    /*Solve NMPC by C/GMRES method*/
    const double current_time = ros::Time::now().toSec(); // Used when first increasing the prediction horizon.
    if (!initial_solution_calculate_)
    {
        // The initial solution is calculated using Newton-GMRES method
        nmpc_solver_ptr_->initializeSolution(current_time, state_vec, path_curvature_, trajectory_speed_, drivable_width_);
        nmpc_solver_ptr_->getControlInput(control_input_vec);
    }
    else
    {
        // Update control_input_vec by C/GMRES method
        const bool mpc_solver_error = nmpc_solver_ptr_->controlUpdate(current_time, state_vec, control_sampling_time_, path_curvature_, trajectory_speed_, drivable_width_, control_input_vec, &control_input_series);
        if (mpc_solver_error)
        {
            ROS_ERROR("[MPC] Break down C/GMRES method.");
            publish_twist(prev_twist_cmd_);
            return;
        }
    }

    /*Publish twist cmd*/
    Twist twist_cmd;
    twist_cmd.x = control_input_vec[MPC_INPUT::TWIST_X];
    twist_cmd.y = 0.0;
    twist_cmd.z = 0.0;
    twist_cmd.roll = 0.0;
    twist_cmd.pitch = 0.0;
    twist_cmd.yaw = control_input_vec[MPC_INPUT::ANGULAR_VEL_YAW];
    publish_twist(twist_cmd);

    prev_twist_cmd_ = twist_cmd;
};

// Update robot pose when subscribe odometry msg
void MPCTracker::callback_odom(const nav_msgs::Odometry &odom)
{
    /*Get current pose via tf*/
    geometry_msgs::TransformStamped trans_form_stamped;
    try
    {
        trans_form_stamped = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    /*update robot pose global*/
    robot_status_.robot_pose_global_.x = trans_form_stamped.transform.translation.x;
    robot_status_.robot_pose_global_.y = trans_form_stamped.transform.translation.y;
    robot_status_.robot_pose_global_.z = trans_form_stamped.transform.translation.z; // not used now
    double roll, pitch, yaw;
    tf2::getEulerYPR(trans_form_stamped.transform.rotation, roll, pitch, yaw);
    robot_status_.robot_pose_global_.roll = roll;   // not used now
    robot_status_.robot_pose_global_.pitch = pitch; // not used now
    robot_status_.robot_pose_global_.yaw = yaw;

    /*update robot twist*/
    robot_status_.robot_twist_.x = odom.twist.twist.linear.x;
    robot_status_.robot_twist_.y = odom.twist.twist.linear.y;
    robot_status_.robot_twist_.z = odom.twist.twist.linear.z;     // not used now
    robot_status_.robot_twist_.roll = odom.twist.twist.angular.x; // not used now
    robot_status_.robot_twist_.pitch = odom.twist.twist.linear.y; // note used now
    robot_status_.robot_twist_.yaw = odom.twist.twist.angular.z;
};

// update reference_course in MPC and calculate curvature
void MPCTracker::callback_reference_path(const nav_msgs::Path &path)
{
    if (path.poses.size() == 0)
    {
        ROS_WARN("Received reference path is empty");
        return;
    }

    /*Set reference course, calculate path curvature and filtering path used in MPC*/
    // TODO::NOTE: NOW SET REFERENCE SPEED IS CONSTANT
    // TODO : リサンプリングがひつようかも
    course_manager_.set_course_from_nav_msgs(path, reference_speed_);

    ROS_INFO("[MPC] Path callback: receive path size = %d", course_manager_.get_mpc_course().size());
};

void MPCTracker::publish_twist(const Twist &twist_cmd) const
{
    geometry_msgs::Twist twist;
    twist.linear.x = twist_cmd.x;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = twist_cmd.roll;
    pub_twist_cmd_.publish(twist);
};