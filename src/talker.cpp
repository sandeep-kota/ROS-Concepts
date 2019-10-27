/**
 * BSD 3-Clause License
 * @copyright (c) 2019, Sandeep Kota
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @file    talker.cpp
 * @author  Sandeep Kota
 * @version 1.0
 * @brief Publisher node
 * @section DESCRIPTION
 * C++ program to publish the topic "chatter" containing custom string message
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

/**
 * @brief Main function implementation to publish to 'chatter' topic and 
 * print the message
 * @param argc Number of parameters passed in command line
 * @param argv Character pointer array pointing to the passed arguments
 * @return 0 When the execution is successful
 */
int main(int argc, char **argv) {
  /// Initialize ROS node
  ros::init(argc, argv, "talker");

  /// Create an instance of NodeHandle
  ros::NodeHandle n;

  // Create a pubisher node to publish 'chatter' topic
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /// Initialize the count
  int count = 0;
  while (ros::ok()) {
    /// Crate string message
    std_msgs::String msg;
    std::stringstream ss;
    ss << "This is a custom message!" << count;
    msg.data = ss.str();

    /// Display the message
    ROS_INFO("%s", msg.data.c_str());

    /// Publish the message using puublisher object
    chatter_pub.publish(msg);

    /// Command to execute all pending callbacks from all nodes
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
