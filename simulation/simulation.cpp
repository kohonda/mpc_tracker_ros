
#include "cgmres_solver/continuation_gmres.hpp"
#include "mpc_tracker/StopWatch.hpp"
#include "mpc_tracker/CSVWriter.hpp"
#include "mpc_tracker/course_manager.hpp"
#include "mpc_tracker/frenet_serret_converter.hpp"
#include "mpc_tracker/mpc_simulator.hpp"
#include "mpc_tracker/Twist.hpp"
#include "mpc_tracker/frenet_state_filter.hpp"

int main()
{
    // // Define the model in NMPC just to use to know the size of optimization formulation
    cgmres::NMPCModel nmpc_model;
    const double sampling_time = 0.1;

    // // Define the solver. Multiple Shooting or Single Shooting
    // // cgmres::ContinuationGMRES nmpc_solver(1.5, 1.0, 100, 1e-08, 100, 15); // default param, Single-Shooting
    // // cgmres::ContinuationGMRES nmpc_solver(1.5, 1.0, 100, 0.002, 1, 3);  // suzlab's param, Single-Shooting
    cgmres::ContinuationGMRES nmpc_solver(1.0, 0.5, 10, 0.002, 1, 100); // honda param, Single-Shooting

    // // Observed info of ego vehicle
    Pose ego_pose_global; // ego pose in global coordinates

    // // Initialize pose and twist of ego car
    ego_pose_global.x = 0.0;
    ego_pose_global.y = 0.0;
    ego_pose_global.z = 0.0;
    ego_pose_global.roll = 0.0;
    ego_pose_global.pitch = 0.0;
    ego_pose_global.yaw = 0.0;

    // // Set the initial guess of the control input
    double solution_initial_guess[nmpc_model.dim_control_input()];
    solution_initial_guess[MPC_INPUT::ANGULAR_VEL_YAW] = 0.01;
    solution_initial_guess[MPC_INPUT::TWIST_X] = 0.01;

    // // Initialize the solution of the C/GMRES method.
    nmpc_solver.setParametersForInitialization(solution_initial_guess, 1e-06, 50);

    // // Set Driving course
    pathtrack_tools::CourseManager course_manager;
    // // The course is given in advance, but it can be updated sequentially.
    // // course_manager.set_course_from_csv("../reference_path/oval_course.csv");
    // // course_manager.set_course_from_csv("../reference_path/circuit_course.csv");
    // // course_manager.set_course_from_csv("../reference_path/step_path_01m.csv");

    course_manager.set_course_from_csv("/home/honda/colcon_ws/src/mpc_tracker_ros/simulation/reference_path/sinwave_cource.csv");
    // // course_manager.set_course_from_csv("../reference_path/stop_course.csv");
    // // course_manager.set_course_from_csv("../reference_path/turn_right.csv");

    std::function<double(double)> course_curvature = [&course_manager](const double &x_f) { return course_manager.get_curvature(x_f); };
    std::function<double(double)> course_speed = [&course_manager](const double &x_f) { return course_manager.get_speed(x_f); };
    std::function<double(double)> drivable_width = [&course_manager](const double &x_f) { return course_manager.get_drivable_width(x_f); };

    // // Frenet Serret Conveter
    pathtrack_tools::FrenetSerretConverter frenet_serret_converter;

    // Reproduce the motion of the control target by numerical integration for simulation
    pathtrack_tools::MPCSimulator mpc_simulator(sampling_time);

    // // State and Control input
    std::vector<double> current_state_vec_frenet(nmpc_model.dim_state()); // current state at Frenet coordinate
    double control_input_vec[nmpc_model.dim_control_input()];             // calculated control input (tire_angle, accel)

    // save result log setting
    const std::string save_csv_file = "/home/honda/colcon_ws/src/mpc_tracker_ros/simulation/simulation_result/result.csv";
    CSVWriter csv;
    // Set header of csv
    /* clang-format off */
    csv.newRow() << "t"
                 << "calc_time[msec]"
                 << "x_g"
                 << "y_g"
                 << "yaw_g"
                 << "twist_x"
                 << "twist_yaw"
                 << "x_f"
                 << "y_f"
                 << "yaw_f"
                 << "F_norm";
    /* clang-format on */
    csv.writeToFile(save_csv_file);

    // // Stop Watch
    StopWatch stop_watch;

    const double start_time = 0.0;
    const double end_time = 100.0;

    // // Simulation loop
    // // When imprement to the real-car, callback inside a loop.
    std::cout << "Start Simulation" << std::endl;
    for (double current_time = start_time; current_time < end_time; current_time += sampling_time)
    {
        // stop watch start
        stop_watch.lap();

        // coordinate convert
        const FrenetCoordinate ego_pose_frenet = frenet_serret_converter.global2frenet(course_manager.get_mpc_course(), ego_pose_global);

        // Set state vector
        current_state_vec_frenet.at(MPC_STATE_SPACE::X_F) = ego_pose_frenet.x_f;
        current_state_vec_frenet.at(MPC_STATE_SPACE::Y_F) = ego_pose_frenet.y_f;
        current_state_vec_frenet.at(MPC_STATE_SPACE::YAW_F) = ego_pose_frenet.yaw_f;

        std::array<std::vector<double>, MPC_INPUT::DIM> control_input_series;
        // Solve by C/GMRES
        if (current_time == start_time)
        {
            // The initial solution is calculated using Newton-GMRES method
            nmpc_solver.initializeSolution(start_time, current_state_vec_frenet, course_curvature, course_speed, drivable_width);
            nmpc_solver.getControlInput(control_input_vec);
        }
        else
        {
            // Update control_input_vec by C/GMRES method
            const bool is_mpc_solve = nmpc_solver.controlUpdate(current_time, current_state_vec_frenet, sampling_time, course_curvature, course_speed, drivable_width, control_input_vec, &control_input_series);
        }

        // stop watch end
        const double calculation_time = stop_watch.lap(); // calculation time at one control interval [msec]

        // Output log data
        /* clang-format off */
        csv.newRow() << current_time
                     << calculation_time
                     << ego_pose_global.x
                     << ego_pose_global.y
                     << ego_pose_global.yaw
                     << control_input_vec[MPC_INPUT::ANGULAR_VEL_YAW]
                     << control_input_vec[MPC_INPUT::TWIST_X]
                     << ego_pose_frenet.x_f
                     << ego_pose_frenet.y_f
                     << ego_pose_frenet.yaw_f
                     << nmpc_solver.getErrorNorm(current_time, current_state_vec_frenet, course_curvature,course_speed,drivable_width);
        /* clang-format on */
        csv.writeToFile(save_csv_file);

        // Pose & Twist update by simulator
        const auto updated_ego_pose_global = mpc_simulator.update_ego_state(current_time, ego_pose_global, control_input_vec, sampling_time);

        ego_pose_global = updated_ego_pose_global;
    }

    std::cout << "End Simulation" << std::endl;

    return 0;
}
