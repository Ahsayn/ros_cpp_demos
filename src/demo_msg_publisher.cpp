/*
 * Copyright (C) 2021, Nyasha Kapfumvuti
 * Email id : nskapf@gmail.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
* This code will publish a integers from 0 to n with a delay of 100ms
 */

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "demo_pkg/demo_msg.h"
#include <iostream>
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "demo_msg_publisher");

    // Create Nodehandl obj
    ros::NodeHandle node_obj;

    // Create Publisher obj
    ros::Publisher custom_msg_publisher = node_obj.advertise<demo_pkg::demo_msg>("/demo_msg_topic", 10);

    // Create Rate obj
    ros::Rate loop_rate(10);
    
    // Initialize message variable
    int number_count = 0;

    // ROS Running Loop
    while(ros::ok())
    {
        // Create custom message
        demo_pkg::demo_msg msg;

        // insert string into custom message
        std::stringstream ss;
        ss << "hello world ";
        msg.greeting = ss.str();

        // Insert variable data into custom message
        msg.number = number_count;

        // Output to ROS_INFO log
        ROS_INFO("%s", msg.greeting.c_str());
        ROS_INFO("%d", msg.number);

        // Publish custom message to topic
        custom_msg_publisher.publish(msg);

        // ROS Update
        ros::spinOnce();

        // Sleep
        loop_rate.sleep();

        // Incr Count
        ++number_count;
    }

    return 0;
}
