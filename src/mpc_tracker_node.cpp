#include "mpc_tracker/mpc_tracker_core.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mpc_tracker");
  MPCTracker mpc_tracker;
  ros::spin();
  return 0;
};
