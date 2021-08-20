cmake_minimum_required(VERSION 3.0.2)
project(mpc_tracker)

add_compile_options(-std=c++17)
add_compile_options(-Wall -Wextra)
add_compile_options(-O3 -march=native -fopenmp)

find_package(catkin REQUIRED COMPONENTS
  # autoware_msgs
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_ros
)

find_package( Eigen3 REQUIRED )
if(${EIGEN3_FOUND})
message("-- Found Eigen version ${EIGEN3_VERSION}: ${EIGEN3_INCLUDE_DIRS}")
else()
message(STATUS "Eigen3 is NOT found")
endif()


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    # autoware_msgs
    geometry_msgs
    nav_msgs
    roscpp
    tf2
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(
  MPC_TRACKER_SRC
  src/course_manager.cpp
  src/frenet_serret_converter.cpp
  # src/mpc_simulator.cpp
  src/frenet_state_filter.cpp
  src/mpc_formulation.cpp
)


set(
  CGMRES_SOLVER_SRC
  src/cgmres_solver/continuation_gmres.cpp
  src/cgmres_solver/single_shooting_continuation.cpp
  src/cgmres_solver/single_shooting_ocp.cpp
  src/cgmres_solver/time_varying_smooth_horizon.cpp
  src/cgmres_solver/cgmres_initializer.cpp
  src/cgmres_solver/zero_horizon_ocp.cpp
  src/cgmres_solver/optimal_control_problem.cpp
  src/cgmres_solver/linear_algebra.cpp
)

add_executable(mpc_tracker_node
  src/mpc_tracker_core.cpp
  src/mpc_tracker_node.cpp
  ${MPC_TRACKER_SRC}
  ${CGMRES_SOLVER_SRC}
) 

add_dependencies(mpc_tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(mpc_tracker_node
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    mpc_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

