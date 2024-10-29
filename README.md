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
    source /opt/ros/jazzy/setup.bash
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

### Running the Project
1. **Source the Workspace**
    Source the setup file for your workspace:
    ```bash
    source ~/ros_sw/install/setup.bash
    ```
2. **Run the Nodes**