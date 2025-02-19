# ROS Setup Instructions

## Install Required Packages

```bash
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-mapviz
sudo apt install ros-jazzy-mapviz-plugins
sudo apt install ros-jazzy-tile-map
```

## Setup ROS Environment

```bash
source /opt/ros/jazzy/setup.bash
```

## Create Workspace and Clone Repositories

```bash
mkdir gps_ws
cd gps_ws
git clone -b 10-gps-navigation https://github.com/KPI-Rover/ros_sw.git
git clone -b jazzy https://github.com/ros-navigation/navigation2_tutorials.git
```

## Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Simulation

```bash
ros2 launch ros_sw launch_sim.launch.py use_rviz:=True use_mapviz:=True
```

## Run GPS Waypoint Follower Demos

```bash
ros2 run nav2_gps_waypoint_follower_demo interactive_waypoint_follower
ros2 run nav2_gps_waypoint_follower_demo gps_waypoint_logger </path/to/yaml/file.yaml>
ros2 run nav2_gps_waypoint_follower_demo logged_waypoint_follower </path/to/yaml/file.yaml>
```

## Run Teleop Twist Keyboard

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=True
```

## More Information

For more information, please visit [Navigation2 with GPS](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html).
