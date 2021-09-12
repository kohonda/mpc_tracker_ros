
#include "cgmres_solver/continuation_gmres.hpp"
#include "mpc_tracker/CSVWriter.hpp"
#include "mpc_tracker/StopWatch.hpp"
#include "mpc_tracker/Twist.hpp"
#include "mpc_tracker/course_manager.hpp"
#include "mpc_tracker/frenet_serret_converter.hpp"
#include "mpc_tracker/frenet_state_filter.hpp"
#include "mpc_tracker/matplotlibcpp.h"
#include "mpc_tracker/mpc_simulator.hpp"

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cout << "The argument is invalid. " << std::endl;
    std::cout << "Usage: ./simulation <input_reference_path.csv> <output_result.csv>" << std::endl;
    return -1;
  }

  const double sampling_time = 0.1;  // control interval [s]

  // Set up C/GMRES solver
  const double Tf = 1.0;
  const double alpha = 0.5;
  const int N = 10;
  const double finite_distance_increment = 0.002;
  const double zeta = 10;
  const int kmax = 10;
  cgmres::ContinuationGMRES nmpc_solver(Tf, alpha, N, finite_distance_increment, zeta, kmax);

  // Observed info of robot pose in global coordinate
  Pose ego_pose_global;
  Twist robot_twist;

  // Initialize pose of robot
  ego_pose_global.x = 0.01;
  ego_pose_global.y = 0.1;
  ego_pose_global.z = 0.0;
  ego_pose_global.roll = 0.0;
  ego_pose_global.pitch = 0.0;
  ego_pose_global.yaw = 0.0;
  robot_twist.x = 0.0;
  robot_twist.y = 0.0;
  robot_twist.yaw = 0.0;

  // Set the initial guess of the control input
  double solution_initial_guess[MPC_INPUT::DIM];
  solution_initial_guess[MPC_INPUT::ANGULAR_VEL_YAW] = 0.01;
  solution_initial_guess[MPC_INPUT::ACCEL] = 0.01;

  // Initialize the solution of the C/GMRES method.
  nmpc_solver.setParametersForInitialization(solution_initial_guess, 1e-06, 100);

  // Set Driving course
  pathtrack_tools::CourseManager course_manager;
  const std::string input_reference_path = static_cast<std::string>(argv[1]);
  course_manager.set_course_from_csv(input_reference_path);

  std::function<double(double)> course_curvature = [&course_manager](const double &x_f) {
    return course_manager.get_curvature(x_f);
  };
  std::function<double(double)> course_speed = [&course_manager](const double &x_f) {
    return course_manager.get_speed(x_f);
  };
  std::function<double(double)> drivable_width = [&course_manager](const double &x_f) {
    return course_manager.get_drivable_width(x_f);
  };

  // Frenet Serret Conveter
  pathtrack_tools::FrenetSerretConverter frenet_serret_converter;

  // Reproduce the motion of the control target by numerical integration for simulation
  pathtrack_tools::MPCSimulator mpc_simulator(sampling_time);

  // State and Control input
  std::vector<double> current_state_vec_frenet(MPC_STATE_SPACE::DIM);  // current state at Frenet coordinate
  double control_input_vec[MPC_INPUT::DIM];                            // calculated control input (tire_angle, accel)

  // save result log setting
  const std::string save_csv_file = static_cast<std::string>(argv[2]);
  CSVWriter csv;
  // Set header of csv
  /* clang-format off */
    csv.newRow() << "t"
                 << "calc_time[msec]"
                 << "x_g"
                 << "y_g"
                 << "yaw_g"
                 << "twist_x"
                 << "input.accel"
                 << "input.twist_yaw"
                 << "x_f"
                 << "y_f"
                 << "yaw_f"
                 << "F_norm";
  /* clang-format on */
  csv.writeToFile(save_csv_file);

  StopWatch stop_watch;

  const double start_time = 0.0;
  const double end_time = 100.0;

  std::cout << "Start Simulation" << std::endl;
  for (double current_time = start_time; current_time < end_time; current_time += sampling_time) {
    // stop watch start
    stop_watch.lap();

    // coordinate convert
    const FrenetCoordinate ego_pose_frenet =
        frenet_serret_converter.global2frenet(course_manager.get_mpc_course(), ego_pose_global);

    // Set state vector
    current_state_vec_frenet.at(MPC_STATE_SPACE::X_F) = ego_pose_frenet.x_f;
    current_state_vec_frenet.at(MPC_STATE_SPACE::Y_F) = ego_pose_frenet.y_f;
    current_state_vec_frenet.at(MPC_STATE_SPACE::YAW_F) = ego_pose_frenet.yaw_f;
    current_state_vec_frenet.at(MPC_STATE_SPACE::TWIST_X) = robot_twist.x;

    std::array<std::vector<double>, MPC_INPUT::DIM> control_input_series;
    // Solve by C/GMRES
    if (current_time == start_time) {
      // The initial solution is calculated using Newton-GMRES method
      nmpc_solver.initializeSolution(start_time, current_state_vec_frenet, course_curvature, course_speed,
                                     drivable_width);
      nmpc_solver.getControlInput(control_input_vec);
    } else {
      // Update control_input_vec by C/GMRES method
      const bool is_mpc_solved =
          nmpc_solver.controlUpdate(current_time, current_state_vec_frenet, sampling_time, course_curvature,
                                    course_speed, drivable_width, control_input_vec, &control_input_series);
      if (!is_mpc_solved) {
        std::cerr << "Break Down C/GMRES Method" << std::endl;
        exit(-1);
      }
    }

    // stop watch end
    const double calculation_time = stop_watch.lap();  // calculation time at one control interval [msec]

    // Output log data
    /* clang-format off */
        csv.newRow() << current_time
                     << calculation_time
                     << ego_pose_global.x
                     << ego_pose_global.y
                     << ego_pose_global.yaw
                     << robot_twist.x
                     << control_input_vec[MPC_INPUT::ACCEL]
                     << control_input_vec[MPC_INPUT::ANGULAR_VEL_YAW]
                     << ego_pose_frenet.x_f
                     << ego_pose_frenet.y_f
                     << ego_pose_frenet.yaw_f
                     << nmpc_solver.getErrorNorm(current_time, current_state_vec_frenet, course_curvature,course_speed,drivable_width);
    /* clang-format on */
    csv.writeToFile(save_csv_file);

    // for real time plot
    std::vector<double> ego_pose_x(1);
    std::vector<double> ego_pose_y(1);
    std::vector<double> ego_yaw_x(2);
    std::vector<double> ego_yaw_y(2);
    if (static_cast<int>(current_time) % 1 == 0) {
      // get predictive state series
      const std::array<std::vector<double>, MPC_STATE_SPACE::DIM> predicted_series =
          mpc_simulator.reproduct_predivted_state(ego_pose_global, robot_twist, control_input_series, sampling_time);

      // clear previous plot
      matplotlibcpp::clf();

      ego_pose_x[0] = ego_pose_global.x;
      ego_pose_y[0] = ego_pose_global.y;
      ego_yaw_x[0] = ego_pose_global.x;
      ego_yaw_y[0] = ego_pose_global.y;
      const double rod_length = 1.0;
      ego_yaw_x[1] = ego_pose_global.x + rod_length * cos(ego_pose_global.yaw);
      ego_yaw_y[1] = ego_pose_global.y + rod_length * sin(ego_pose_global.yaw);

      matplotlibcpp::named_plot("Reference path", course_manager.get_mpc_course().x, course_manager.get_mpc_course().y,
                                "g:");
      matplotlibcpp::named_plot("Robot position", ego_pose_x, ego_pose_y, "or");
      // matplotlibcpp::plot(ego_yaw_x, ego_yaw_y, "r");
      matplotlibcpp::named_plot("Planned path", predicted_series[MPC_STATE_SPACE::X_F],
                                predicted_series[MPC_STATE_SPACE::Y_F], "b-");

      matplotlibcpp::xlim(ego_pose_global.x - 5, ego_pose_global.x + 5);
      matplotlibcpp::ylim(ego_pose_global.y - 5, ego_pose_global.y + 5);

      // Add graph title
      matplotlibcpp::title("Real Time Plotter");
      // Enable legend.
      matplotlibcpp::legend();

      // Display plot continuously
      matplotlibcpp::pause(0.01);
    }

    // Pose update by simulator
    const auto [updated_ego_pose_global, updated_robot_twist] =
        mpc_simulator.update_ego_state(current_time, ego_pose_global, robot_twist, control_input_vec, sampling_time);

    ego_pose_global = updated_ego_pose_global;
    robot_twist = updated_robot_twist;
  }

  std::cout << "End Simulation" << std::endl;

  return 0;
}
