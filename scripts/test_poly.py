#!/usr/bin/env python

""" Example code of how to move a robot around the shape of a square. """

import math 
import numpy as np
import sys, argparse

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import tf

class MotionDemos():
  def __init__(self):
    print ('Motion Demo class initialised.')

  def simpleDemo(self, current_pose):
    path = JointTrajectory()

    path.header.frame_id = "world"
    path.joint_names = ['x','y','z','phi','j5','j6']

    # Start
    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0],current_pose[1],current_pose[2],0.0, 0.0]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)

    # Goal
    pxy = [0.5, 0.0]
    (nx, ny) = rot2d(pxy[0], pxy[1], current_pose[2])

    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0] + nx, current_pose[1] + ny, current_pose[2], 0., -1.57]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)

    return path

  def singleBendDemo(self, current_pose):
    path = JointTrajectory()

    path.header.frame_id = "world"
    path.joint_names = ['x','y','phi','j5','j6']

    # Start
    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0],current_pose[1],current_pose[2],0.0, 0.0]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)
    
    # First waypoint - robot extended
    pxy = [0.5, 0.]
    (nx, ny) = rot2d(pxy[0], pxy[1], current_pose[2])

    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0] + nx, current_pose[1] + ny, current_pose[2], 1.2, -1.2]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)

    # Second waypoint - robot in the middle of bend
    pxy = [0.75, 0.25]
    (nx, ny) = rot2d(pxy[0], pxy[1], current_pose[2])

    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0] + nx, current_pose[1] + ny, current_pose[2] + 1.571/2., 1.2, -0.9]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)

    # Third waypoint - robot in the middle of bend
    pxy = [1.0, 0.5]
    (nx, ny) = rot2d(pxy[0], pxy[1], current_pose[2])

    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0] + nx, current_pose[1] + ny, current_pose[2] + 1.571, 1.3, -1.2]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)

    # Goal
    pxy = [1.0, 1.0]
    (nx, ny) = rot2d(pxy[0], pxy[1], current_pose[2])

    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0] + nx, current_pose[1] + ny, current_pose[2] + 1.571, 0., 0.]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)

    return path
  
  def secondDemo(self, current_pose):
    path = JointTrajectory()

    path.header.frame_id = "world"
    path.joint_names = ['x','y','phi','j5','j6']

    # Start
    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0],current_pose[1],current_pose[2],0.0, 0.0]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)
    
    # First waypoint - robot extended
    pxy = [0.4, 0.]
    (nx, ny) = rot2d(pxy[0], pxy[1], current_pose[2])

    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0] + nx, current_pose[1] + ny, current_pose[2], 0.0, -1.571]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)

    # Second waypoint - robot in the middle of bend
    pxy = [0.8, 0.]
    (nx, ny) = rot2d(pxy[0], pxy[1], current_pose[2])

    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0] + nx, current_pose[1] + ny, current_pose[2], 1.571/2., -1.571/2.]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)

    # Third waypoint - robot in the middle of bend
    pxy = [1.0, 0.]
    (nx, ny) = rot2d(pxy[0], pxy[1], current_pose[2])

    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0] + nx, current_pose[1] + ny, current_pose[2], 1.571, 0.]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)

    # Goal
    pxy = [1.3, 0.3]
    (nx, ny) = rot2d(pxy[0], pxy[1], current_pose[2])

    q_point = JointTrajectoryPoint()
    q_point.positions = [current_pose[0] + nx, current_pose[1] + ny, current_pose[2] + 1.571, 0., 0.]
    q_point.velocities = [0.0]*5
    path.points.append(q_point)

    return path

class PathGenerator:
  def __init__(self,rate):
    rospy.init_node('Waypoint_Generator') 		# Initialises node
    
    self.pub_traj_ = rospy.Publisher('/waypoints', JointTrajectory, queue_size=10)

    rospy.sleep(0.5)

  def testPolynomial(self):

    z_offset = 0.

    path = JointTrajectory()

    path.header.frame_id = "world"
    path.joint_names = ['x','y','z','phi','j5','j6']

    # Start
    q_point = JointTrajectoryPoint()
    q_point.positions = [0.0, -0.0, z_offset, 1.57, 0.0, 0.0]
    q_point.velocities = [0.0]*5
    q_point.time_from_start = rospy.Duration(0.0)
    path.points.append(q_point)
    
    # Waypoint 1
    q_point = JointTrajectoryPoint()
    q_point.positions = [0.5, 0.0, z_offset, 0.0, 0.1, -0.4]
    q_point.velocities = [0.0]*5
    q_point.time_from_start = rospy.Duration(0.4)
    path.points.append(q_point)

    # Waypoint 2
    q_point = JointTrajectoryPoint()
    q_point.positions = [1.0, 0.0, z_offset, -1.57, 0.5, -0.8]
    q_point.velocities = [0.0]*5
    q_point.time_from_start = rospy.Duration(0.7)
    path.points.append(q_point)

    for i in range(0,1):
      self.pub_traj_.publish(path)
      rospy.sleep(0.1)

if __name__ == "__main__":
  
  print 'Initialising Waypoint Generator ....'
  manager = PathGenerator(25)
  rospy.loginfo('Generator Ready!')

  manager.testPolynomial()