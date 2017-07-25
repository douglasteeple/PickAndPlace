# Robotics Arm: Pick & Place

### Forward and Inverse Kinematics - The Amazon Challenge Pick and Place Robot Kinematics project.

## Introduction

The object of the Pick and Place project is to write inverse kinematics formulae that implement the Amazon Pick and Place challenge, to correctly pick an item off a shelf at a random location and place it in a bin. To succeed the implementation must do this 8 out of 10 times.

Kinematics refers to position in space without regard to weights or forces. Inverse kinematics in this project involves the calculation of robot joint angles to position a Kuka KR210 robot arm at the position of an item so it can be grasped and follow a trajectory to release the item in a bin. Inverse kinematics is challenging as there are multiple solutions to the problem.

The project involves calculating the inverse kinemtics equations and modifying the file IK_server.py to implement the solution in the context of an RVIZ/gazebo environment that has been set up by the course creators.

This image shows the gazebo environment with the shelf, the items (blue cylinders) and the Kuka robot arm in the simulation environment:

![project environment](images/RobotPicknPlace-Jun19.jpg)

Full documentation can be found here: 

<a href="https://github.com/douglasteeple/PickAndPlace/PickAndPlace.pdf" target=_blank>PackAndPlace.pdf</a>

## The Process






