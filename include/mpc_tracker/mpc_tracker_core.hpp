#ifndef MPC_TRACKER_Core_H
#define MPC_TRACKER_Core_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>
#include <mutex>
#include "Twist.hpp"
#include "mpc_tracker/course_manager.hpp"
#include "mpc_tracker/frenet_serret_converter.hpp"
// #include "mpc_tracker/frenet_state_filter.hpp"
#include "StopWatch.hpp"
#include "cgmres_solver/continuation_gmres.hpp"
#include "mpc_tracker/mpc_simulator.hpp"

/**
 * @class reference trajectory tracker class
 * @brief calculate control input based nonlinear MPC using C/GMRES method
 * @author Kohei Honda <0905honda@gmail.com>
 * @date Aug.2021
 */
class MPCTracker {
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
  std::mutex mtx_;  //!< @brief variable for lock/unlock of updating class member variables
  /*ros system variables*/
  ros::NodeHandle nh_;            //!< @brief ros public node handle
  ros::NodeHandle private_nh_;    //!< @brief ros private node handle
  ros::Publisher pub_twist_cmd_;  //!< @brief twist command topic publisher
  ros::Subscriber sub_ref_path_;  //!< @brief reference path subscriber
  ros::Subscriber sub_odom_;      //!< @brief robot odom subscriber
  ros::Timer timer_control_;      //!< @brief timer for control command commutation
  std::string robot_frame_id_;
  std::string map_frame_id_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /*control system parametes*/
  double control_sampling_time_;  //!< @brief control interval [s]
  double reference_speed_;        //!< @brief robot reference speed [m/s]

  /*path smoothing parameters*/
  int curvature_smoothing_num_;       //!< @brief Smoothing value for curvature calculation
  double max_curvature_change_rate_;  //!< @brief Saturate value for curvature change rate [1/m^2]
  double speed_reduction_rate_;  //!< @brief Reduce the speed reference based on the rate of curvature change; v_ref' =
                                 //!< v_ref * exp (-speed_reduction_rate * curvature_rate^2)
  double deceleration_rate_for_stop_;  //!< @brief Reduce the speed reference for stopping; v_ref'  = v_ref * (1 -
                                       //!< exp(-deceleration_rate_for_stop * (x_goal -x_f))), recommend the same value
                                       //!< as a_min in MPC formulation

  /*cgmres solver parameters*/
  struct CGMRESParam {
    double Tf_;                         //!< @brief Length of predictive horizon in C/GMRES [s]
    double alpha_;                      //!< @brief The length horizon at time t is given by T_f * (1-exp(-alpha*t))
    int N_;                             //!< @brief The number of discrete of the predictive horizon
    double finite_distance_increment_;  //!< @brief Step length of the finite difference in C/GMRES method
    double zeta_;                       //!< @brief A parameter for stabilization of the C/GMRES method.
    int kmax_;                          //!< @brief dimension of the Krylov subspace and maximum iteration number
  };
  CGMRESParam cgmres_param_;

  /*MPC parameters*/
  struct MPCParam {
    std::array<double, MPC_STATE_SPACE::DIM> q_;           //!< @brief Weight of state for stage cost in MPC
    std::array<double, MPC_STATE_SPACE::DIM> q_terminal_;  //!< @brief Weight of state for terminal cost in MPC
    std::array<double, MPC_INPUT::DIM> r_;                 //!< @brief Weight of input for stage cost in MPC
    double barrier_coefficient_;  //!< @brief barrier function coefficient for inequality constraint
    double a_max_;                //!< @brief Maximum acceleration
    double a_min_;                //!< @brief Minimum deceleration
  };
  MPCParam mpc_param_;

  /*Variables*/
  struct RobotStatus {
    Pose robot_pose_global_;              //!< @brief Global robot pose used in MPC
    Twist robot_twist_;                   //!< @brief Robot twist used in MPC
    FrenetCoordinate robot_pose_frenet_;  //!< @brief Frenet-Serret robot pose used in MPC
  };
  RobotStatus robot_status_;

  Twist prev_twist_cmd_;  //!< @brief published twist command just before

  /*function used in the predictive horizon of MPC*/
  std::function<double(double)> path_curvature_;  //!< @brief return curvature from pose x_f in frenet coordinate
  std::function<double(double)>
      trajectory_speed_;                          //!< @brief return reference speed from pose x_f in frenet coordinate
  std::function<double(double)> drivable_width_;  // not used now

  /*Flags*/
  bool is_robot_state_ok_ = false;           //!< @brief Check getting robot observed info
  bool initial_solution_calculate_ = false;  //!< @brief Initialize C/GMRES method using Newton-method
  bool is_finish_goal_ = false;              //!< @brief Check reached the goal
  bool is_permit_control_ = false;           //!< @brief Check control permission from higher level planner

  /*Visualization info*/
  ros::Publisher pub_predictive_pose_;   //! @brief MPC predictive pose publisher
  ros::Publisher pub_calculation_time_;  //! @brief calculation time publisher
  ros::Publisher pub_F_norm_;            //! @brief F norm is Deviation from optimal solution
  ros::Publisher pub_twist_x_;           //! @brief pub odom twist_x for visualization
  ros::Publisher pub_cmd_twist_x_;       //! @brief pub command twist_x for visualization
  ros::Publisher pub_twist_yaw_;         //! @brief pub odom twist_x for visualization
  ros::Publisher pub_cmd_twist_yaw_;     //! @brief pub command twist_x for visualization

  /*Library*/
  std::unique_ptr<cgmres::ContinuationGMRES> nmpc_solver_ptr_;  //!< @brief nonlinear mpc solver pointer
  std::unique_ptr<pathtrack_tools::CourseManager>
      course_manager_ptr_;  //!< @brief Manage reference path, reference speed and drivable area in MPC
  pathtrack_tools::FrenetSerretConverter
      frenet_serret_converter_;  //!< @brief Converter between global coordinate and frenet-serret coordinate
  std::unique_ptr<pathtrack_tools::MPCSimulator> mpc_simulator_ptr_;  //!< @brief Reproduct MPC predictive state
  StopWatch stop_watch_;
  // std::unique_ptr<pathtrack_tools::FrenetStateFilter> frenet_state_filter_ptr_; //!< @brief Frenet state estimate
  // from observed info

  /**
   * @brief calculate and publish control input
   *
   */
  void timer_callback(const ros::TimerEvent &);

  /**
   * @brief Subscribe to odometry and update pose and twist at that time. Pose is updated from tf, and twist is updated
   * from odometry.
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
  visualization_msgs::MarkerArray convert_predictivestate2marker(
      const std::array<std::vector<double>, MPC_STATE_SPACE::DIM> &predictive_state, const std::string &name_space,
      const std::string &frame_id, const double &r, const double &g, const double &b, const double &z) const;

  /**
   * @brief
   *
   * @param control_input_vec
   * @param control_input_series
   * @return true : Success of Optimization
   * @return false : Failure of Optimization
   */
  bool calculate_mpc(std::array<double, MPC_INPUT::DIM> *control_input,
                     std::array<std::vector<double>, MPC_INPUT::DIM> *control_input_series, double *F_norm);

  /**
   * @brief Reset C/GMRES method when break down
   *
   * @param solution_initial_guess
   */
  void reset_cgmres(const std::array<double, MPC_INPUT::DIM> &solution_initial_guess);
};

#endif
