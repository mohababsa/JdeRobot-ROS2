# GSoC 2025 ROS Challenge Progress

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
  Output: `Publishing: "Hello! ROS2 is fun." every second`
- Terminal 2:
  ```bash
  ros2 run hello_ros2 subscriber_node
  ```
  Output: `I heard: "Hello! ROS2 is fun." in sync`

4. **Demo**
   Video shows publisher sending messages and subscriber receiving them
   
## Author
**Mohamed ABABSA** - GSoC 2025 Applicant

## License
TaskMaster is licensed under the [MIT License](LICENSE)

