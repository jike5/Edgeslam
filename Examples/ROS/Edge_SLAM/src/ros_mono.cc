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

#include"../../../include/System.h"

// Edge-SLAM
#include <string>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    // Edge-SLAM: check arguments
    // Check run type and convert to lowercase
    std::string RunType(argv[3]);
    std::transform(RunType.begin(), RunType.end(), RunType.begin(), ::tolower);
    ros::init(argc, argv, RunType);
    ros::start();
    if((argc != 4) || ((RunType.compare("client") != 0) && (RunType.compare("server") != 0)))
    {
        cerr << endl << "Usage: rosrun Edge_SLAM Mono VOC_PATH SETTINGS_PATH RUN_TYPE(client|server)" << endl;
        ros::shutdown();
        return 1;
    }

    // Edge-SLAM
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],RunType,ORB_SLAM2::System::MONOCULAR,true);

    // Edge-SLAM: check client or server
    if (RunType.compare("client") == 0)
    {
        ImageGrabber igb(&SLAM);

        ros::NodeHandle nodeHandler;

        ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

        ros::spin();

        // Edge-SLAM: split shutdown between client and server
        // Stop all threads
        SLAM.ClientShutdown();
    }
    else
    {
        ros::spin();

        // Edge-SLAM: split shutdown between client and server
        // Stop all threads
        SLAM.ServerShutdown();

        // Save camera trajectory
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

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
}

