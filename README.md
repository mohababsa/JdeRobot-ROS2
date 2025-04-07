# GSoC 2025 ROS2 Challenge Progress

## Overview
This README documents my progress on the GSoC 2025 ROS challenge as of April 6, 2025. The challenge consists of three parts:
- Part 1a: ROS2 Publisher-Subscriber Demo.
- Part 1b: TurtleBot3 Simulation with Laser Scan Visualization.
- Part 2: ROS2 Navigation2 with Waypoint Navigation.
Below, I detail the setup, development, successes, and challenges for each part, showcasing my work with ROS2 Humble on an Ubuntu 22.04 VM in VMware Workstation.

## Environment Setup
- Platform: Ubuntu 22.04 (Jammy) in VMware Workstation.
- ROS2 Version: Humble Hawksbill.
- VM Configuration: 4 GB RAM, 2 CPUs, 512 MB graphics memory, 3D acceleration enabled.

## Part 1a: ROS2 Publisher-Subscriber Demo
1. **Objective**
Create a ROS2 package with a publisher and subscriber to demonstrate basic ROS2 communication.

2. **Development**
Package creation ![Sign Up](screenshots/scr1.png)

3. **Build and Run**
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
- Terminal 1:
  ```bash
  ros2 run hello_ros2 publisher_node
  ```
  Output: `Publishing: "Hello! ROS2 is fun."` every second
- Terminal 2:
  ```bash
  ros2 run hello_ros2 subscriber_node
  ```
  Output: `I heard: "Hello! ROS2 is fun."` in sync

4. **Demo**
   Video shows publisher sending messages and subscriber receiving them

## Part 1b: TurtleBot3 Simulation with Laser Scan Visualization
1. **Objective**
Simulate TurtleBot3 in Gazebo and visualize its laser scan in RViz2

2. **Install Dependencies**
   ```bash
   sudo apt install ros-humble-turtlebot3-simulations ros-humble-gazebo-ros-pkgs -y
   ```

3. **Launch Attempt**
   ```bash
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

4. **RViz2 Setup**
   ```bash
   ros2 run rviz2 rviz2
   ```

5. **Challenges**
- Gazebo Issue: `gzserver` starts but `/spawn_entity` service fails:
- Error: `Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?`
- Standalone `ros2 run gazebo_ros gzserver` exits with code 255.

6. **Status**
- In Progress: Gazebo integration unresolved; TurtleBot3 not spawning.

## Part 2: ROS2 Navigation2 with Waypoint Navigation
1. **Objective**
Navigate TurtleBot3 in Gazebo using Navigation2, hitting at least three waypoints.

2. **Install Dependencies**
   ```bash
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
   ```
   
3. **Development**
Package creation ![Sign Up](screenshots/scr1.png)

4. **Build and Run**
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
- Terminal 1: Gazebo
  ```bash
  source /opt/ros/humble/setup.bash
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  ```
  Issue: `/spawn_entity` unavailable.
- Terminal 2: Navigation2
  ```bash
  ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
  ```
  Issue: `KeyError: 'TURTLEBOT3_MODEL'`.
- Terminal 3: Waypoint Navigator
  ```bash
  ros2 run waypoint_nav waypoint_navigator
  ```
  Output: `Sending waypoint 1: (1.0, 0.0, 0.0)` (hangs without Navigation2).

5. **Challenges**
- Gazebo: Same `/spawn_entity` issue as Part 1b.
- Navigation2: Missing environment variable fixed by adding `export TURTLEBOT3_MODEL=burger`.

6. **Status**
- Success: `waypoint_nav` package built and runs.
- In Progress: Gazebo and Navigation2 integration pending.


## Author
**Mohamed ABABSA** - GSoC 2025 Applicant

## License
TaskMaster is licensed under the [MIT License](LICENSE)

