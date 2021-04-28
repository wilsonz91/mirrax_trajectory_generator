/*
 * Trajectory generator node for MIRRAX robot
 *
 *Polynomial node: takes in a set of waypoints of type JointTrajectory
  and generate a polynomial trajectory, up to SNAP continuity
 */

#include  "ros/ros.h"
#include <mirrax_trajectory_generator/polynomial.h>

#include <iostream>

int main(int argc, char** argv) {

  ros::init(argc, argv, "polynomial_node");

  ros::NodeHandle n;
  Polynomial planner(n);
  
  ros::AsyncSpinner spinner(1); // Use 1 thread
	spinner.start();
	ros::waitForShutdown();

  return 0;
}