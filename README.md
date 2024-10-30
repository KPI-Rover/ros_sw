# ROS 2 Jazzy Project Setup

This project is built using **ROS 2 Jazzy**. This guide will walk you through the steps required to clone, build, and run the project on a ROS 2 Jazzy workspace.

---

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building the Workspace](#building-the-workspace)
- [Running the Project](#running-the-project)

---

### Prerequisites

Ensure you have the following installed:

- **ROS 2 Jazzy**: [Installation guide](https://docs.ros.org/en/jazzy/Installation.html)
- **colcon**: Build tool for ROS 2

### Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/KPI-Rover/ros_sw.git
   ```

2. **Install Dependencies**
    ```bash
    cd ~/ros_sw
    rosdep install --from-paths src --ignore-src -r -y
    ```

### Building the Workspace
1. **Source the ROS 2 Jazzy Environment**
    Make sure the ROS 2 Jazzy environment is sourced before building the workspace:
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```
2. **Build with colcon**
    ```bash
    colcon build
    ```
3. **Start robot**
    ```bash
    ros2 launch apricotka-robot-car launch_sim.launch.py
    ```
4. **Keyboard control**
    ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=True -p frame_id:=base_link
    ```