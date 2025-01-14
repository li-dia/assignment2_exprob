# ROS Action Interface for Marker Detection and Waypoint Navigation

## Overview
This assignment focuses on implementing a robot control system in a simulated environment using ROS and PDDL (Planning Domain Definition Language). The objective is to enable a robot to autonomously navigate between predefined waypoints, detect ArUco markers, and identify the waypoint associated with the smallest detected marker ID.

The task integrates **motion planning**, **marker detection**, and **automated decision-making** through the following key components:

- **ROSPlan Integration**: Utilizes ROSPlan for planning and executing tasks such as navigation and marker detection.  
- **ArUco Marker Detection**: Detects marker IDs and their positions using custom ROS topics.  
- **Waypoint Navigation**: Allows the robot to traverse predefined waypoints based on planner-generated goals.  
- **PDDL Actions**: Defines logical actions (`move_to`, `detect_marker`, `go_to_least_id`) for high-level task automation.

This project demonstrates the application of robotics concepts like task planning, perception, and autonomous navigation in a Gazebo-simulated environment. It provides a practical approach to bridging planning and execution in robotics.

## Features
- **Waypoint Navigation**: The robot can navigate to predefined waypoints on a map based on ROSPlan action requests.
- **Marker Detection**: The system can detect markers using camera information and determine the marker with the least ID.
- **Action Interface**: The project integrates with ROSPlan's action dispatch system, allowing for interaction with action servers for autonomous decision-making.

## Prerequisites
- ROS (Robot Operating System) installed
- ROSPlan setup and dependencies
- `move_base` action server available for waypoint navigation
- ROS message types: `geometry_msgs`, `std_msgs`, `move_base_msgs`, `rosplan_dispatch_msgs`

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/repository-name.git
   ```
2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
   ```
## Usage
### Runnig the node

1. Make sure SLAM_packages and ROSPlan are also be cloned:

```bash
git clone https://github.com/CarmineD8/SLAM_packages.git # Remember to switch to noetic branch
git clone https://github.com/KCL-Planning/ROSPlan.git
  ```

2. To start, run the following command:
```bash
   roslaunch assignment2_exprob assignment2.launch
   ```
3. Open a separate terminal and execute the following command to initiate the planning process and dispatch the plan:
```bash
   roslaunch assignment2_exprob planning.launch
   ```
4. In a separate terminal, execute the following commands to generate the plan, parse it, and dispatch it sequentially:
 ```bash
rosservice call /rosplan_problem_interface/problem_generation_server  
rosservice call /rosplan_planner_interface/planning_server  
rosservice call /rosplan_parsing_interface/parse_plan  
rosservice call /rosplan_plan_dispatcher/dispatch_plan  
```
### PDDL Actions: 
1. Move_to: Moves the robot from one waypoint to another.
2. Detect_marker: Detects ArUco markers in the robot's vicinity.
3. Go_to_least_id: Navigates to the waypoint linked to the smallest detected marker ID.

