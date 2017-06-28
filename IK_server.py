#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from IK_calcs import *

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            
	        # px,py,pz = end-effector position
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            
            # roll, pitch, yaw = end-effector orientation
            lx = req.poses[x].orientation.x
            ly = req.poses[x].orientation.y
            lz = req.poses[x].orientation.z
            lw = req.poses[x].orientation.w

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([lx, ly,lz, lw])
     
            # Calculate joint angles using Geometric IK method

            theta1, theta2, theta3, theta4, theta5, theta6 = IK_calcs(px, py, pz, roll, pitch, yaw)
            
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
