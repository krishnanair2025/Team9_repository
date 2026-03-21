# Robotic System Design - Team 9
## Introduction:

This repository contains the design, development and implementation files for Team 9's autonomous rover system, created as part of the AERO62520 module.

## Project Overview:

Our rover is being designed to autonomously explore a 6m x 4m area containing randomly placed obstacles, identify and collect three coloured cylinders (red,green and yellow) and return them to the colour-coded bins located at the start position.

To achieve this, the team is structured into sub-teams:
+ Navigation and Mapping - Responsible for developing the navigation capabilities of the rover along with the task manager which orchestrates the behaviour of the robot during the mission.
+ Object Perception - Responsible for developing the object detection and localisation capabilities of the rover.
+ Manipulation - Responsible for developing the manipulator's trajectory planning and grasping capabilities. 

## Repository Structure

The repository contains directories for the team's documents, including the updated workplace charter, Design Review Analysis and Preliminary Design Review. The packages directory holds each sub-team's ROS2 workspace.

## Team members:
+ Krishna Nair (Navigation and Mapping sub-team, Payload sled design)
+ Drago Jakimovski (Manipulation sub-team)
+ Mengjie Zhang (Manipulation sub-team)
+ Lun Li (Object Perception sub-team)
+ Lexin Wang (Navigation and Mapping sub-team)
+ Mochi Zhang (Object Perception sub-team)

## Navigation Simulation

The following video shows a preview of the working navigation simulation in Gazebo. It operates using the mission state machine (task_manager_node) and a simple openCV based green object detecting camera node. It simulates the manipulator working by using a random timer as there is no simulated manipulator in this setup. This simulation is capable of demonstrating how the rover explores the map and approaches objects that it detects until 3 objects have been collected. Simulation of the sorting phase of the mission is still pending.

https://github.com/user-attachments/assets/b19657b1-ff3d-47d2-9826-172c5d89c94a


