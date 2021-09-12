# mpc_tracker_ros
Motion Controller for trajectory tracking Using Nonlinear Model Predictive Control (MPC) with C/GMRES Method.

Control target: two-wheel differential drive mobile robot

Outputted control input: twist.x and twist.yaw

## Requirements

- Ubuntu 18.04 or 20.04
- gcc 7.50 or higher
- cmake 3.0.2 or higher
- ROS Melodic or Noetic


## Third Party

- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [C/GMRES Solver](https://github.com/mayataka/autogenu-jupyter)
- [CSV Writer](https://github.com/al-eax/CSVWriter) (for offline simulation)
- [CSV Parser](https://github.com/d99kris/rapidcsv) (for offine simulation)
- [matplotlib-cpp](https://github.com/lava/matplotlib-cpp) (for offline simulation)



## Pre-Installation

```bash
sudo apt update

sudo apt install libeigen3-dev

sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin

sudo apt install -y python3-colcon-common-extensions

sudo apt install python3-matplotlib, python3-numpy

```



## How to Build

```bash
mkdir ~catkin_ws/src -p

cd catkin_ws/src

git clone git@github.com:kohonda/mpc_tracker_ros.git

cd ..

rosdep update

rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
```



## How to test by offline simulation

```bash
cd catkin_ws/src

./mpc_tracker_ros/scripts/offline_simulation.sh
```

Then, start simulation with real-time plot and get the simulation result at `mpc_tracker_ros/simulation/simulation_result/result.csv` after termination.

![offline_simulation_movie_sinwave](https://user-images.githubusercontent.com/50091520/132941168-dd29277c-3ea1-484d-8265-42da788f6a4e.mp4)


## How to launch ROS node

```bash
cd catkin_ws

source install/setup.bash

roslaunch mpc_tracker mpc_tracker.launch
```
#### I/O

```bash
Node [/mpc_tracker]
Publications: 
 * /twist_cmd [geometry_msgs/Twist] : calculated control input (twist.yaw, twist.x)
 * /mpc_tracker/F_norm [std_msgs/Float32] : Deviation from KKT conditions
 * /mpc_tracker/calculation_time [std_msgs/Float32]
 * /mpc_tracker/cmd_twist_x [std_msgs/Float32] : for visualization
 * /mpc_tracker/cmd_twist_yaw [std_msgs/Float32] : : for visualization
 * /mpc_tracker/predictive_pose [visualization_msgs/MarkerArray] : for visualization
 * /mpc_tracker/robot_twist_x [std_msgs/Float32] : for visualization
 * /mpc_tracker/robot_twist_yaw [std_msgs/Float32] : for visualization
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /odom [nav_msgs/Odometry] : observed Twist of the robot
 * /reference_path [nav_msgs/Path] : reference path in map coordinate
 * /tf [tf2_msgs/TFMessage] : observed Pose of the robot in map coordinate
```

