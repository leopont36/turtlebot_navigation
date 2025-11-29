# Intelligent robotics assignment 1 - group n. 18
## Setup
1. Clone this repository inside the `/ws_18_assignments/src` folder:
  ```bash
  cd ~/ws_18_assignments/src
  git clone git@github.com:marcofacco2001/group18_assignment_1.git
  ```
2. Compile the packages:
  ```bash
  cd ..
  colcon build --packages-select group18_mission_control group18_interfaces
  ```
3. Run launch file:
  ```bash
  source install/setup.bash
  ros2 launch group18_mission_control group18_assignment_1.launch.py
  ```
