# Tele operation Server for ROS2 Turtlebot3

This repository contains:

 - TurtleBot3 Tele Operation Robotic Client
 - Tele Operation Server
 - Tele Operation UI


## Tele operation client

### Build

```
$ mkdir -P ~/teleop_ws/src
$ cd ~/teleop_ws/src
$ git clone https://github.com/atolab/fog05_ros2_demo_teleop
$ cd ..
$ source /opt/ros/dashing/setup.bash
$ colcon build --symlink-install
```

### Run

```
$ source /opt/ros/dashing/setup.bash
$ source ~/teleop_ws/install/setup.bash
$ ros2 run turtlebot3_teleop teleop_keyboard
```


## Tele Operation Server

The tele operation server is designed to be packaged as a Docker container, it comes with two docker files for `amd64` and `arm64` architecture

### Build

```
$ cd teleop-docker
$ sg docker -c "docker build ./ -f ./Dockerfile -t teleop-server --no-cache" --oom-kill-disable
```

### Run

```
$ docker run -ip 5000:5000 --env ZENOH=<zenoh router ip> --name teleop teleop-server
```


## Tele Operation GUI

The tele operation WebUI is designed to be packaged as a Docker container, it comes with two docker files for `amd64` and `arm64` architecture.

You may need to update the IP address of the server in the UI, update variable `global_url` in file `gui/js/robot-global.js`

### Build

```
$ cd gui-docker
$ sg docker -c "docker build ./ -f ./Dockerfile -t teleop-gui --no-cache" --oom-kill-disable
```

### Run

```
$ docker run -ip 80:80 --name gui gui-server
```

