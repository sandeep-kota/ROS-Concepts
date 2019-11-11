<h1 align="center"> ROS Tutorial: Subscriber/Publisher Node with Service and Logging
</h1>
ENPM808x-Programming Assignment ROS Services, Logging and Launch files

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Introduction

This is a simple publisher-subscriber node implementation using ROS. 

## Project Overview

- Talker: Publishes message "Initial Message" as a topic named "chatter". The talker node also has a service named "update_string" that will update the string being published by the "talker" node. The talker node also broadcasts a tf transform between `/world` and `/talk` frames. 
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
cd beginner_tutorials/
git checkout Week11_HW
cd ../..  
catkin_make
```

## Run Instructions

1) To launch the launch file of the talker-subscriber node with the service, run the following command.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials talker_listener.launch frequency:=5
#Where frequency is a positive number with default value as 10Hz
```
2) To run the service node, run the following command in a new terminal
```
cd ~/catkin_ws/
source devel/setup.bash
rosservice call /update_string "Your New Message To Publish!"
```
## Rosbag Instructions

1) To launch the subscriber publisher node with rosbag node to collect data, use flag `rosbagRecorder:=true` flag at the end of roslaunch command.
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials talker_listener.launch frequency:=5 rosbagRecorder:=true
#Where frequency is a positive number with default value as 10Hz
```
2) To replay the rosbag data collected, run the following command while the listener node is running in another terminal.
```
cd ~/catkin_ws/
source devel/setup.bash
cd ~/catkin_ws/src/beginner_tutorials/Results/
rosbag play recorder.bag 
```

## TF Inspection Instructions

The tools mentioned below are tools that can be used to debug the tf frames being broadcasted by the nodes.

1) To view the tf frames being broadcasted by the `talker` node, run the following command. This will show a graph of how the frames are connected.
```
cd ~/catkin_ws/
source devel/setup.bash
cd ~/catkin_ws/src/beginner_tutorials/Results/
rosrun tf view_frames
evince frames.pdf
```

2) To vizualize the frames broadcasting in runtime, use `rqt_tf_tree` by running the following command
```
cd ~/catkin_ws/
source devel/setup.bash
rosrun rqt_tf_tree rqt_tf_tree
```

3) To view the transform between the `/world` frame and the /`talk` frame, use `tf_echo`.
```
cd ~/catkin_ws/
source devel/setup.bash
rosrun tf tf_echo world talk
```

## Test Running Instructions

1) To run the unit test cases, we use the `catkin_make run_tests_beginner_tutorials` command. Before running this command, it should be ensured that all instances of this projet are terminated. 
```
cd ~/catkin_ws/
catkin_make run_tests_beginner_tutorials
``` 