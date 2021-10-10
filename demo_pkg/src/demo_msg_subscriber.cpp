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
#include "demo_pkg/demo_msg.h"
#include <iostream>
#include <sstream>

// Topic update callback
void custom_msg_callback(const demo_pkg::demo_msg::ConstPtr& msg)
{
    // Output data to ROS_INFO log
    ROS_INFO("Received greeting: [%s]", msg->greeting.c_str());
    ROS_INFO("Received number: [%d]", msg->number);
}

// Main Function
int main(int argc, char **argv)
{
    // Initialize ROS node with node name
    ros::init(argc, argv, "demo_msg_subscriber");

    // Init Node Handle
    ros::NodeHandle node_obj;

    // Init Subscriber with topic name, buffer size, callback function
    ros::Subscriber demo_msg_subsciber = node_obj.subscribe("/demo_msg_topic", 10, custom_msg_callback);

    // ROS Update
    ros::spin();

    return 0;
}
