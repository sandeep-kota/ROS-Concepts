<h1 align="center"> ROS Tutorial: Subscriber/Publisher Node with Service and Logging
</h1>
ENPM808x-Programming Assignment ROS Services, Logging and Launch files

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Introduction

This is a simple publisher-subscriber node implementation using ROS. 

## Project Overview

- Talker: Publishes message "Initial Message" as a topic named "chatter". The talker node also has a service named "update_string" that will update the string being published by the "talker" node. 
- Listener: Subscribes to the topic "chatter" and prints message on the terminal window.

## Dependencies

This package has been tested in a system with following dependencies.
- Ubuntu 16.04 LTS
- ROS-Kinetic distro

## Build instructions

1) To install ROS-Kinetic follow the steps mentioned in the official website (http://wiki.ros.org/kinetic/Installation/Ubuntu)
2) After installing ROS-Kinetic run the following commands to download this project.
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone https://github.com/sandeep-kota/beginner_tutorials.git
git checkout Week10_HW
cd ~/catkin_ws/  
catkin_make
```

## Run Instructions

1) To launch the launch file of the talker-subscriber node with the service, run the following command.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials/talker frequency:= 5 
#Where frequency is a positive number with default value as 10
```
2) To run the service node, run the following command in a new terminal
```
cd ~/catkin_ws/
source devel/setup.bash
rosservice call /update_string "Your New Message To Publish!"
```


