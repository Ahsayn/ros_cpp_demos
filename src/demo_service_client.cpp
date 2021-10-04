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
#include "demo_pkg/demo_srv.h"
#include <iostream>
#include <sstream>

using namespace std;


// Main Function
int main(int argc, char **argv)
{
    // Init ROS node with name arg
    ros::init(argc, argv, "demo_service_client");
    
    // Init Nodehandle obj
    ros::NodeHandle node_obj;

    // Loop Rate
    ros::Rate loop_rate(10);

    // Init Server
    ros::ServiceClient client = node_obj.serviceClient<demo_pkg::demo_srv>("demo_service");

    // ROS Running loop
    while(ros::ok())
    {
        demo_pkg::demo_srv srv;
        std::stringstream ss;
        ss << "Sending from Client";
        srv.request.in = ss.str();

        if (client.call(srv))
        {
            // Log req
            ROS_INFO("From Client  [%s], Server says [%s]",srv.request.in.c_str(),srv.response.out.c_str());
        }
        else
        {
            // log error
            ROS_ERROR("Failed to call service :(");
            return 1;
        }

        ros::spinOnce();

        // Sleep
        loop_rate.sleep();
    }
    return 0;
}