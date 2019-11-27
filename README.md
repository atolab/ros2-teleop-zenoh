# Teleoperation Server for ROS2 Turtlebot3

This repository contains the teleoperation server

## Build

```
$ mkdir -P ~/teleop_ws/src
$ cd ~/teleop_ws/src
$ git clone https://github.com/atolab/fog05_ros2_demo_teleop
$ cd ..
$ source /opt/ros/dashing/setup.bash
$ colcon build --symlink-install
```

## Run

```
$ source /opt/ros/dashing/setup.bash
$ source ~/teleop_ws/install/setup.bash
$ ros2 run turtlebot3_teleop teleop_keyboard
```

