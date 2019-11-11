cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

# Compile using C++11 standard
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg message_generation tf)

## Add service files
add_service_files(FILES updateString.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

## Create a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Add executable for talker node
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_gencpp)

## Add executable for talker node
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

## Add executables for rostest and gtest
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
  talkTest
  test/talkTest.launch
  test/talkTest.cpp
  )

  target_link_libraries(
  talkTest
  ${catkin_LIBRARIES}
  )

  add_dependencies(
  talkTest
  talker
  listener
  ${catkin_EXPORTED_TARGETS}
  )
endif()
