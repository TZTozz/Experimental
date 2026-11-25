# Repository Structure

This repository contains two assignments for the Experimental Robotics Laboratory course:
- **Assignment 1:** ArUco Treasure Hunt
- **Assignment 2**

Each assignment is split into two branches:
- `simulation`: For testing the robot in the RViz and Gazebo simulators.
- `realRobot`: For deploying and testing the code on the physical ROSbot.

## Assignment 1 – ArUco Treasure Hunt

Full assignment details are available in the `simulation` branch, whereas the `realRobot` branch focuses specifically on the modifications required to adapt the simulation code for the physical hardware.

Both branches share the same structure and contain three packages:
- `custom_msgs`: Defines the custom ROS messages used in the project.
- `erl1`: The main project package, containing robot models, configuration files, launch files, and nodes.
- `ros_aruco_opencv`: Handles ArUco marker detection.