cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

## Create a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Add executable for talker node
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

## Add executable for listener node
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})