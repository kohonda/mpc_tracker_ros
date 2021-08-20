# mpc_tracker_ros
Path tracking controller package by Nonlinear MPC using C/GMRES method

## Requirements
- Ubuntu 18.04 or higher
- gcc
- Eigen3
- cmake 3.0.2 or higher

## Reference
- [C/GMRES Solver](https://github.com/mayataka/autogenu-jupyter)

## Pre-Installation

```bash
sudo apt update

sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin

sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool

pip3 install -U setuptools
```

## How to Build

```bash
mkdir ~catkin_ws/src -p

cd catkin_ws/src

git clone git@github.com:kohonda/mpc_tracker_ros.git

rosdep update

rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
```

## Parameters

