#include "mpc_tracker/mpc_tracker_core.hpp"

MPCTracker::MPCTracker() : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_), prev_twist_cmd_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
{
    //  Set parameters from ros param
    /*control parameters*/
    private_nh_.param("control_sampling_time", control_sampling_time_, static_cast<double>(0.1));
    private_nh_.param("reference_speed", reference_speed_, static_cast<double>(1.0));
    private_nh_.param("zero_speed_points_around_goal", zero_speed_points_around_goal_, static_cast<int>(1));

    /*cgmres parameters*/
    private_nh_.param("Tf", cgmres_param_.Tf_, static_cast<double>(1.0));
    private_nh_.param("alpha", cgmres_param_.alpha_, static_cast<double>(0.5));
    private_nh_.param("N", cgmres_param_.N_, static_cast<int>(10));
    private_nh_.param("finite_distance_increment", cgmres_param_.finite_distance_increment_, static_cast<double>(0.0002));
    private_nh_.param("zeta", cgmres_param_.zeta_, static_cast<double>(62.5));
    private_nh_.param("kmax", cgmres_param_.kmax_, static_cast<int>(5));

    /*mpc parameters*/
    mpc_param_.q_.at(MPC_STATE_SPACE::X_F) = 0.0;
    private_nh_.param("weight_lat_error", mpc_param_.q_.at(MPC_STATE_SPACE::Y_F), static_cast<double>(1.0));
    private_nh_.param("weight_yaw_error", mpc_param_.q_.at(MPC_STATE_SPACE::YAW_F), static_cast<double>(1.0));
    mpc_param_.q_terminal_.at(MPC_STATE_SPACE::X_F) = 0.0;
    private_nh_.param("terminal_weight_lat_error", mpc_param_.q_terminal_.at(MPC_STATE_SPACE::Y_F), static_cast<double>(1.0));
    private_nh_.param("terminal_weight_yaw_error", mpc_param_.q_terminal_.at(MPC_STATE_SPACE::YAW_F), static_cast<double>(1.0));
    private_nh_.param("weight_input_angular_yaw", mpc_param_.r_.at(MPC_INPUT::ANGULAR_VEL_YAW), static_cast<double>(1.0));
    private_nh_.param("weight_input_twist_x", mpc_param_.r_.at(MPC_INPUT::TWIST_X), static_cast<double>(1.0));
    // TODO: max, min of input

    /*Initialize C/GMRES Solver */
    nmpc_solver_ptr_ =
        std::make_unique<cgmres::ContinuationGMRES>(cgmres_param_.Tf_, cgmres_param_.alpha_, cgmres_param_.N_, cgmres_param_.finite_distance_increment_, cgmres_param_.zeta_, cgmres_param_.kmax_);
    const double solution_initial_guess[MPC_INPUT::DIM] = {0.01, 0.01};
    nmpc_solver_ptr_->setParametersForInitialization(solution_initial_guess, 1e-06, 50);
    nmpc_solver_ptr_->continuation_problem_.ocp_.model_.set_parameters(mpc_param_.q_, mpc_param_.q_terminal_, mpc_param_.r_);

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

    /*Set publisher for visualization*/
    mpc_simulator_ptr_ = std::make_unique<pathtrack_tools::MPCSimulator>(control_sampling_time_);
    pub_predictive_pose_ = private_nh_.advertise<visualization_msgs::MarkerArray>("predictive_pose", 1);
    pub_calculation_time_ = private_nh_.advertise<std_msgs::Float32>("calculation_time", 1);
    pub_F_norm_ = private_nh_.advertise<std_msgs::Float32>("F_norm", 1);

    /*Service Client*/
    service_control_trigger_ = nh_.advertiseService("follow_path_cmd", &MPCTracker::callback_control_trigger, this);
    client_finish_goal_ = nh_.serviceClient<autoware_msgs::Trigger>("goal_reached_report");
};

MPCTracker::~MPCTracker()
{
    ROS_INFO("Destoruct MPC tracker node");
    Twist stop_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    publish_twist(stop_twist);
};

void MPCTracker::timer_callback([[maybe_unused]] const ros::TimerEvent &te)
{
    // TODO: ゴール判定 -> report
    // const double dist_to_goal =

    /*Guard higher level planner permit*/
    if (!is_permit_control_)
    {
        Twist zero_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        ROS_INFO("[MPC] Waiting higher level planner permission for control");
        publish_twist(zero_twist);
        return;
    }

    /*Guard finishing goal*/
    if (!is_finish_goal_)
    {
        Twist zero_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        ROS_INFO("[MPC] Finishing goal");
        publish_twist(zero_twist);

        return;
    }

    /*Guard robot and reference path status*/
    const int path_size = course_manager_.get_path_size();
    if (path_size == 0 || !is_robot_state_ok_)
    {
        ROS_INFO("[MPC] MPC is not solved. ref_path_size: %d, is_pose_set: %d", path_size, is_robot_state_ok_);
        publish_twist(prev_twist_cmd_);
        return;
    }

    stop_watch_.lap();

    /*coordinate convert from euclid to frenet-serret*/
    robot_status_.robot_pose_frenet_ = frenet_serret_converter_.global2frenet(course_manager_.get_mpc_course(), robot_status_.robot_pose_global_);

    /*Set state vector*/
    const std::vector<double> state_vec{robot_status_.robot_pose_frenet_.x_f, robot_status_.robot_pose_frenet_.y_f, robot_status_.robot_pose_frenet_.yaw_f};

    /*Control input*/
    double control_input_vec[MPC_INPUT::DIM] = {0.0};

    /*Control input series for visualization */
    std::array<std::vector<double>, MPC_INPUT::DIM> control_input_series;

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
        const bool mpc_solver_error =
            nmpc_solver_ptr_->controlUpdate(current_time, state_vec, control_sampling_time_, path_curvature_, trajectory_speed_, drivable_width_, control_input_vec, &control_input_series);
        if (mpc_solver_error)
        {
            ROS_ERROR("[MPC] Break down C/GMRES method.");
            publish_twist(prev_twist_cmd_);
            return;
        }
    }

    const double calculation_time = stop_watch_.lap();

    /*NAN Guard*/
    if (!std::isnan(control_input_vec[MPC_INPUT::TWIST_X]) || !std::isnan(control_input_vec[MPC_INPUT::ANGULAR_VEL_YAW]))
    {
        ROS_ERROR("[MPC] Calculate NAN control input");
        publish_twist(prev_twist_cmd_);
        return;
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

    /*Visualization*/
    const std::array<std::vector<double>, MPC_STATE_SPACE::DIM> predictive_poses =
        mpc_simulator_ptr_->reproduct_predivted_state(robot_status_.robot_pose_global_, control_input_series, control_sampling_time_);
    const visualization_msgs::MarkerArray markers = convert_predictivestate2marker(predictive_poses, "mpc_predictive_pose", map_frame_id_, 0.99, 0.99, 0.99, 0.2);
    pub_predictive_pose_.publish(markers);

    std_msgs::Float32 calculation_time_msg;
    calculation_time_msg.data = static_cast<float>(calculation_time);
    pub_calculation_time_.publish(calculation_time_msg);

    std_msgs::Float32 F_norm_msg;
    const double F_norm = nmpc_solver_ptr_->getErrorNorm(current_time, state_vec, path_curvature_, trajectory_speed_, drivable_width_);
    F_norm_msg.data = static_cast<float>(F_norm);
    pub_F_norm_.publish(F_norm_msg);
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
    course_manager_.set_course_from_nav_msgs(path, reference_speed_, static_cast<int>(zero_speed_points_around_goal_));

    ROS_INFO("[MPC] Path callback: receive path size = %d", course_manager_.get_mpc_course().size());
};

bool MPCTracker::callback_control_trigger([[maybe_unused]] autoware_msgs::Trigger::Request &request, autoware_msgs::Trigger::Response &response)
{
    is_permit_control_ = true;
    response.received = true;
    ROS_INFO("[MPC] Receive control permission from supervisor");

    return true;
}

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

visualization_msgs::MarkerArray MPCTracker::convert_predictivestate2marker(const std::array<std::vector<double>, MPC_STATE_SPACE::DIM> &predictive_state, const std::string &name_space,
                                                                           const std::string &frame_id, const double &r, const double &g, const double &b, const double &z) const
{
    visualization_msgs::MarkerArray marker_array;

    /*Add line*/
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = name_space + "/line";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.15;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 0.9;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    for (size_t i = 0; i < predictive_state.at(0).size(); i++)
    {
        geometry_msgs::Point p;
        p.x = predictive_state.at(MPC_STATE_SPACE::X_F).at(i);
        p.y = predictive_state.at(MPC_STATE_SPACE::Y_F).at(i);
        p.z = z;
        marker.points.push_back(p);
    }

    /*Add pose as arrow*/
    visualization_msgs::Marker marker_poses;
    marker_poses.header.frame_id = frame_id;
    marker_poses.header.stamp = ros::Time();
    marker_poses.ns = name_space + "/poses";
    marker_poses.lifetime = ros::Duration(0.5);
    marker_poses.type = visualization_msgs::Marker::ARROW;
    marker_poses.action = visualization_msgs::Marker::ADD;
    marker_poses.scale.x = 0.2;
    marker_poses.scale.y = 0.1;
    marker_poses.scale.z = 0.2;
    marker_poses.color.a = 0.99; // Don't forget to set the alpha!
    marker_poses.color.r = r;
    marker_poses.color.g = g;
    marker_poses.color.b = b;

    for (size_t i = 0; i < predictive_state.at(0).size(); i++)
    {
        marker_poses.id = i;
        marker_poses.pose.position.x = predictive_state.at(MPC_STATE_SPACE::X_F).at(i);
        marker_poses.pose.position.y = predictive_state.at(MPC_STATE_SPACE::Y_F).at(i);
        marker_poses.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, predictive_state.at(MPC_STATE_SPACE::YAW_F).at(i));
        marker_poses.pose.orientation = tf2::toMsg(q);
        marker_array.markers.push_back(marker_poses);
    }

    return marker_array;
}

void MPCTracker::report_reached_goal()
{
    /*flag true*/
    is_finish_goal_ = true;

    /*Report to supervisor*/
    autoware_msgs::Trigger goal_reached_srv_call;
    goal_reached_srv_call.request.trigger = true;
    if (client_finish_goal_.call(goal_reached_srv_call))
    {
        bool reset_flag = goal_reached_srv_call.response.received;
        if (reset_flag == true)
        {
            ROS_INFO("[MPC] Goal reached completely");
            is_permit_control_ = false;
        }
    }
    else
    {
        ROS_ERROR("[MPC] Goal Communication to job supervisor FAILED");
    }
}
