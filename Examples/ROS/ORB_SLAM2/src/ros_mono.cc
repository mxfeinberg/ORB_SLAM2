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
//#include "../../../include/MapPoint.h"
#include <std_msgs/Float32MultiArray.h>
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){
    	// zmq::context_t context(1);
    	// context_ = &context;
    	// zmq::socket_t publisher(*context_, 3);
    	// publisher_ = &publisher;
    	// publisher_->bind("tcp://10.193.157.243:9000");

    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
    ros::NodeHandle nh;
    ros::Publisher pub;

    ros::NodeHandle nh2;
    ros::Publisher pub2;
    // zmq::context_t *context_;
    // zmq::socket_t *publisher_;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);
    igb.pub = igb.nh.advertise<std_msgs::Float32MultiArray>("obstacles", 1);
    igb.pub2 = igb.nh2.advertise<std_msgs::Float32MultiArray>("position", 1);
    // zmq::context_t context(1);
    // zmq::socket_t publisher(context, ZMQ_PUB);
    // publisher.bind("tcp://*:5563");

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);
    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());


    const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpSLAM->GetTrackedMapPoints();
    const cv::Point3f curr_pos = mpSLAM->getFrame();
    
    if(vpRefMPs.empty()) 
      return;
    set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
    //vector<cv::Point3f> obst;
    std_msgs::Float32MultiArray array;
    std_msgs::Float32MultiArray position;
    // ROS_INFO("attempting to publish");
    for(set<ORB_SLAM2::MapPoint*>::iterator  sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++ )
    {
      if(*sit == NULL || (*sit)->isBad())
        continue;
      cv::Mat pos =  (*sit)->GetWorldPos();
      
      	array.data.push_back(pos.at<float>(0));
      //if(!(pos.at<float>(1) == NULL) )
        array.data.push_back(pos.at<float>(1));
      //if(!(pos.at<float>(2) == NULL) )
        array.data.push_back(pos.at<float>(2)); 
        //ROS_INFO("SUFFICIENT DATA");
    }
    pub.publish(array);
    
    if(&curr_pos != NULL) {
    	position.data.push_back(curr_pos.x);
    	position.data.push_back(curr_pos.y);
    	position.data.push_back(curr_pos.z);
    	//ROS_INFO("Location Confirmed");
    }
    pub2.publish(position);
    
    ros::spinOnce();

    // s_sendmore(*publisher_, "A");
    // s_send(*publisher_, "We Don't want to see this.");
    // s_sendmore(*publisher_, "B");
    // s_send(*publisher_, "We Don't want to see this.");
}