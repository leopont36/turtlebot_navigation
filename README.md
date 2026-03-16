# TurtleBot3 Autonomous Navigation with LiDAR — ROS2

Autonomous mobile robot navigation implemented in ROS2, developed as part of 
the Intelligent Robotics course. The robot navigates to a goal position given by two apritag poses using 
`navigate_to_pose`. When the turlebot detects by the Lidar two parallel wals (corridor), it start to navigate by using velocity contros and the Lidar Feedback to adjust trajectory and avoid the wals, At the end of navigation it detect the position of circular obstacles by using the lidar

## Features
- Goal-oriented navigation with `navigate_to_pose` action
- Real-time velocity control with LiDAR feedback for wall avoidance
- Circular obstacle detection from LiDAR point data

## Technologies
- ROS2
- TurtleBot3
- Nav2
- LiDAR
- Gazebo (simulation environment provided by course instructor)

## Report
The full project report is available [here](docs/report.pdf)


## Demo
https://github.com/user-attachments/assets/d7d75f08-7ce8-4939-9dbb-6ca27631d755
[Watch high quality version on Google Drive](https://drive.google.com/file/d/1Dbm9LEF050eERIH8sGPA6Jor6-NZKNz8/view?usp=drive_link)


## Setup

> This project requires a Gazebo simulation environment provided by the 
> course instructor (not included in this repository).

1. Clone this repository inside your workspace:
```bash
   cd ~/ws_18_assignments/src
   git clone https://github.com/leopont36/turtlebot_navigation.git
```
2. Build the packages:
```bash
   cd ..
   colcon build --packages-select group18_mission_control group18_interfaces
```
3. Run the launch file:
```bash
   source install/setup.bash
   ros2 launch group18_mission_control group18_assignment_1.launch.py
```

## Contributors
Developed in collaboration with [@marcofacco2001](https://github.com/marcofacco2001) and [@giaco-mas](https://github.com/giaco-mas)
and other group members as part of a university assignment.
