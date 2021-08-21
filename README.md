# mpc_tracker_ros
Trajectory Tracking Controller for ROS Package Using Nonlinear Model Predictive Control (MPC) with C/GMRES Method

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

sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin

sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool

sudo apt install python3-matplotlib, python3-numpy

pip3 install -U setuptools
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

## How to run offline simulation

```bash
cd catkin_ws/src

./mpc_tracker_ros/scripts/offline_simulation.sh
```

Then, start simulation with real-time plot and get the simulation result at `mpc_tracker_ros/simulation/simulation_result/result.csv` after termination.

[sample movie](movie/sinwave05.gif)

## How to launch online node

## Parameters

