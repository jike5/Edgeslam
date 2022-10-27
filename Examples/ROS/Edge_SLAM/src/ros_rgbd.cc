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
#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"

// Edge-SLAM
#include <string>

using namespace std;

bool save_traj = false;

bool save_traj_callback(
    std_srvs::Empty::Request &request, 
    std_srvs::Empty::Response &response
) {
    save_traj = true;
    ROS_INFO("Save traj");
    return true;
}

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    // Edge-SLAM: check arguments
    // Check run type and convert to lowercase
    std::string RunType(argv[3]);
    std::transform(RunType.begin(), RunType.end(), RunType.begin(), ::tolower);
    // Kubeedge: check communication protocol
    std::string CommType(argv[4]);
    std::transform(CommType.begin(), CommType.end(), CommType.begin(), ::tolower);

    ros::init(argc, argv, RunType);
    ros::start();
    if((argc != 5) || ((RunType.compare("client") != 0) && (RunType.compare("server") != 0)))
    {
        cerr << endl << "Usage: rosrun Edge_SLAM RGBD VOC_PATH SETTINGS_PATH RUN_TYPE(client|server) COMM_TYPE(tcp|websocket)" << endl;
        ros::shutdown();
        return 1;
    }

    // kubeedge: Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], RunType, CommType, ORB_SLAM2::System::RGBD, true);

    // Edge-SLAM: check client or server
    if (RunType.compare("client") == 0)
    {
        ImageGrabber igb(&SLAM);
        
        // Trajectory file
        ofstream f;
        f.open("/CameraTrajectory.txt");
        f.close();

        ros::NodeHandle nh;

        message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
        message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 1);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
        message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
        sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

        ros::spin();

        // Edge-SLAM: split shutdown between client and server
        // Stop all threads
        SLAM.ClientShutdown();
    }
    else
    {
        ros::NodeHandle nh;
        // register service save_traj:
        ros::ServiceServer service = nh.advertiseService("save_traj", save_traj_callback);

        ros::spin();

        // Edge-SLAM: split shutdown between client and server
        // Stop all threads
        SLAM.ServerShutdown(save_traj);

        // Save camera trajectory
        // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory-edgeslam.txt");
    }

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Trajectory file
    ofstream f;
    f.open("/CameraTrajectory.txt", ios::app);
    f << fixed;

    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    // kubeedge: save trajectory
    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
    f << setprecision(6) << cv_ptrRGB->header.stamp.toSec() << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    f.close();
}

