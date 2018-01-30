/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

//#include <zmq.hpp>
#include "zhelpers.hpp"

#include <std_msgs/Float32MultiArray.h>
// class ImageGrabber
// {
// public:
//     ImageGrabber(){
//         // zmq::context_t context(1);
//         // context_ = &context;
//         // zmq::socket_t publisher(*context_, 3);
//         // publisher_ = &publisher;
//         // publisher_->bind("tcp://10.193.157.243:9000");

//     }

//     void GrabImage(const sensor_msgs::ImageConstPtr & msg);

// };
// void chatterCallback(const std_msgs::String::ConstPtr& msg) {
//     ROS_INFO("WUT");
// }
void obstCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
void posCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
//vector<float> data;
zmq::context_t* _context;
zmq::socket_t* _publisher;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "communicate");
    ros::start();
    //zmq::context_t context(1);
    _context = new zmq::context_t(1);
    //zmq::socket_t publisher(context, ZMQ_PUB);
    _publisher = new zmq::socket_t(*_context, ZMQ_PUB);
    _publisher->bind("tcp://10.193.157.243:5563");
    //ImageGrabber igb();
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("obstacles", 1, obstCallback);
    ros::NodeHandle nh2;
    ros::Subscriber sub2 = nh2.subscribe("position", 1, posCallback);
    ros::spin();
    ros::shutdown();
    return 0;
}
void obstCallback(const std_msgs::Float32MultiArray::ConstPtr& array) {
    s_sendmore(*_publisher, "obstacles");
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); it++)
    {
        if(*it == NULL)
            continue;
        char array[10];
        sprintf(array, "%f", *it);
        s_sendmore(*_publisher, array);
    }
    s_send(*_publisher, "End o");
}

void posCallback(const std_msgs::Float32MultiArray::ConstPtr& array) {
    ROS_INFO("got here");
    s_sendmore(*_publisher, "position");
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); it++)
    {
        if(*it == NULL)
            continue;
        char array[10];
        sprintf(array, "%f", *it);
        s_sendmore(*_publisher, array);
    }
    s_send(*_publisher, "End p");
}


// void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
// {
//     // Copy the ros image message to cv::Mat.
//     return;
// }
