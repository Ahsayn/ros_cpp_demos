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

// Include Headers

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

// Main Function
int main(int argc, char **argv) 
{
    // Init ROS node with name
    ros::init(argc, argv, "demo_topic_publisher");

    // Node handler object
    ros::NodeHandle node_obj;

    // Set up publiser object with handler, topic name and buffer size
    ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers", 10);

    // Publish rate object
    ros::Rate loop_rate(10);

    // Variable to be published
    int number_count = 0;

    // ROS main running loop
    while(ros::ok())
    {
        // Create integer message
        std_msgs::Int32 msg;

        // Insert data into message
        msg.data = number_count;

        // Publish data
        number_publisher.publish(msg);

        // Log data in ROS_INFO Log
        ROS_INFO("%d", msg.data);

        // Update task
        ros::spinOnce();

        // Sleep
        loop_rate.sleep();

        // Increment integer message
        ++number_count;
    }

    return 0;
}