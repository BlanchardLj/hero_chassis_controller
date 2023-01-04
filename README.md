# hero_chassis_controller

## Overview

PID is used to control the speed of wheels, inverse kinematics is used to calculate the expected speed of each wheel, 
forward kinematics is used to realize the odometer, and keyboard is used to control the chassis.

**Keywords:** PID, inverse kinematics, odometer, keyboard


### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Blanchard Lj<br />**

The hero_chassis_controller package has been tested under [ROS] Indigo, Melodic and Noetic on respectively Ubuntu 14.04, 18.04 and
20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


[comment]: <> (### Publications)

[comment]: <> (If you use this work in an academic context, please cite the following publication&#40;s&#41;:)

[comment]: <> (* P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart: **PAPER TITLE**. IEEE/RSJ International Conference)

[comment]: <> (  on Intelligent Robots and Systems &#40;IROS&#41;, 2015. &#40;[PDF]&#40;http://dx.doi.org/10.3929/ethz-a-010173654&#41;&#41;)

[comment]: <> (        @inproceedings{Fankhauser2015,)

[comment]: <> (            author = {Fankhauser, P\'{e}ter and Hutter, Marco},)

[comment]: <> (            booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems &#40;IROS&#41;},)

[comment]: <> (            title = {{PAPER TITLE}},)

[comment]: <> (            publisher = {IEEE},)

[comment]: <> (            year = {2015})

[comment]: <> (        })

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- hardware_interface
- forward_command_controller
- pluginlib
- control_toolbox
- geometry_msgs
- control_msgs
- realtime_tools
- tf
- nav_msgs
#### Install dependencies:
  sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone https://github.com/BlanchardLj/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

### Running in Docker

Docker is a great way to run an application with all dependencies and libraries bundles together. Make sure
to [install Docker](https://docs.docker.com/get-docker/) first.

First, spin up a simple container:

	docker run -ti --rm --name ros-container ros:noetic bash

This downloads the `ros:noetic` image from the Docker Hub, indicates that it requires an interactive terminal (`-t, -i`)
, gives it a name (`--name`), removes it after you exit the container (`--rm`) and runs a command (`bash`).

Now, create a catkin workspace, clone the package, build it, done!

	apt-get update && apt-get install -y git
	mkdir -p /ws/src && cd /ws/src
	git clone https://github.com/BlanchardLj/hero_chassis_controller.git
	cd ..
	rosdep install --from-path src
	catkin_make
	source devel/setup.bash
	roslaunch hero_chasssis_controller hero_chassis_controller.launch

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch hero_chasssis_controller hero_chassis_controller.launch

## Config files


* **default.yaml** Params of joint_state_controller , four PID controllers, 
wheelbase and track width of chassis.


## Launch files

* **hero_chassis_controller.launch:** standard simulation of hero chassis

## Nodes

### hero_chassis_controller

Read the speed and calculate and control the chassis movement.

#### Subscribed Topics

* **`/cmd_vel`** ([geometry_msgs/Twist])

  Speed command issued to chassis.

#### Published Topics

* **`/controller/hero_chassis_controller/state
  `** ([control_msgs/JointControllerState])

  Publish the state of the joint controller.

* **`/controller/hero_chassis_controller/odom`** ([nav_msgs/Odometry])

  Calculate the speed of the chassis according to the actual speed of the wheels, and realize the odometer by superposition.




#### Parameters

* **`wheel_R`** (double, default: 0.0675)

  Radius of wheels.

* **`wheel_track`** (double, default: 0.4)

  Track width of chassis.

* **`wheel_base`** (double, default: 0.4)

  Wheelbase of chassis.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/BlanchardLj/hero_chassis_controller/issues)
.


[ROS]: http://www.ros.org

[rviz]: http://wiki.ros.org/rviz

[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html

[geometry_msgs/Twist]: http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html

[nav_msgs/Odometry]: http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html

[control_msgs/JointControllerState]: http://docs.ros.org/en/api/control_msgs/html/msg/JointControllerState.html