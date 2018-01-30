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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include "../../../include/System.h"

using namespace std;
using namespace cv;

// class ImageGrabber
// {
// public:
//     ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

//     void GrabImage(const sensor_msgs::ImageConstPtr& msg);

//     ORB_SLAM2::System* mpSLAM;
// };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_streamer");
    ros::start();
    
    
    ros::NodeHandle nodeHandler;
    image_transport::ImageTransport it(nodeHandler);
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);
    
    String URL = "http://172.16.107.207:8080/stream/video.mjpeg";
    VideoCapture vid;
    if(!vid.open(URL)) cout << "Shits fucked yo";   
    
    Mat img;
    vid.read(img);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

    ros::Rate loop_rate(20);
    while(nodeHandler.ok()) {
        vid.read(img);
        if(!img.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    //ros::Publisher image_pub = nodeHandler.advertise

    // ImageGrabber igb(&SLAM);

    // ros::NodeHandle nodeHandler;
    // ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    // ros::spin();

    // // Stop all threads
    // SLAM.Shutdown();

    // // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    vid.release();
    ros::shutdown();

    return 0;
}

// void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
// {
//     // Copy the ros image message to cv::Mat.
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvShare(msg);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
// }


