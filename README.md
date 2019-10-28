<h1 align="center"> ROS Tutorial: Subscriber/Publisher Node
</h1>
ENPM808x-Programming Assignment

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Introduction

This is a simple publisher-subscriber node implementation using ROS. 

## Project Overview

- Talker: Publishes message "This is a test message!" as a topic named "chatter".
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
cd ~/catkin_ws/  
catkin_make
```

## Run Instructions

1) Start the `roscore` in a terminal window.
```
source /opt/ros/kinetic/setup.bash
roscore
```
2) Run the `talker` node of the `beginner_tutorials` package in another terminal window.
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```
3) Run the `listener` node of the `beginner_tutorials` package in another terminal window.
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
