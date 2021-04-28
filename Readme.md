# Overview
Trajectory generator for the MIRRAX reconfigurable robot. The trajectory generation has been adapted from the [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) package for this robot. 

## How to use
After building, launch the included launch file:

    rosrun mirrax_trajectory_generator main

The trajectory node is executed and waits for setpoints. The initial start state, waypoints and goal state are all expected to be provided via ROS topic to the callback `/waypoints`. The trajectory can be visualized in RViZ under the `trajectory_markers` topic.

Setting the parameter `zero_waypoint_velocity` switches the trajectory generator to either have C1 (True) or C3 (False) continuity.
