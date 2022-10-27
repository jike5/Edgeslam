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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<Converter.h>
#include<System.h>

// Edge-SLAM
#include <string>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    // kubeedge
    if(argc < 5)
    {
        cerr << endl << "Client Usage: ./rgbd_tum VOC_PATH SETTINGS_PATH RUN_TYPE(client|server) COMMUNICATION_PROTOCOL(websocket|tcp) SEQUENCE_PATH ASSOCIATION_PATH" << endl;
        cerr << endl << "Server Usage: ./rgbd_tum VOC_PATH SETTINGS_PATH RUN_TYPE(client|server) COMMUNICATION_PROTOCOL(websocket|tcp)" << endl;
        return 1;
    }

    // Edge-SLAM: check arguments
    // Check run type and convert to lowercase
    std::string RunType(argv[3]);
    std::transform(RunType.begin(), RunType.end(), RunType.begin(), ::tolower);
    // Kubeedge: check communication protocol
    std::string CommType(argv[4]);
    std::transform(CommType.begin(), CommType.end(), CommType.begin(), ::tolower);
    if(RunType.compare("client") == 0)
    {
        // kubeedge
        if(argc != 7)
        {
            cerr << endl << "Client Usage: ./rgbd_tum VOC_PATH SETTINGS_PATH RUN_TYPE(client|server) COMMUNICATION_PROTOCOL(websocket|tcp) SEQUENCE_PATH ASSOCIATION_PATH" << endl;
            cerr << endl << "Server Usage: ./rgbd_tum VOC_PATH SETTINGS_PATH RUN_TYPE(client|server) COMMUNICATION_PROTOCOL(websocket|tcp)" << endl;
            return 1;
        }

        // Retrieve paths to images
        vector<string> vstrImageFilenamesRGB;
        vector<string> vstrImageFilenamesD;
        vector<double> vTimestamps;
        string strAssociationFilename = string(argv[6]);
        LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

        // Check consistency in the number of images and depthmaps
        int nImages = vstrImageFilenamesRGB.size();
        if(vstrImageFilenamesRGB.empty())
        {
            cerr << endl << "No images found in provided path." << endl;
            return 1;
        }
        else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
        {
            cerr << endl << "Different number of images for rgb and depth." << endl;
            return 1;
        }

        // Edge-SLAM
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        ORB_SLAM2::System SLAM(argv[1],argv[2],RunType, CommType, ORB_SLAM2::System::RGBD,true);

        // Vector for tracking time statistics
        vector<float> vTimesTrack;
        vTimesTrack.resize(nImages);

        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl << endl;

        // Main loop
        cv::Mat imRGB, imD;
        // Trajectory file
        ofstream f;
        f.open("CameraTrajectory.txt");
        f << fixed;

        for(int ni=0; ni<nImages; ni++)
        {
            // Read image and depthmap from file
            imRGB = cv::imread(string(argv[5])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
            imD = cv::imread(string(argv[5])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestamps[ni];

            if(imRGB.empty())
            {
                cerr << endl << "Failed to load image at: "
                    << string(argv[4]) << "/" << vstrImageFilenamesRGB[ni] << endl;
                return 1;
            }

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            // Pass the image to the SLAM system
            cv::Mat Tcw = SLAM.TrackRGBD(imRGB,imD,tframe);

            // kubeedge: save trajectory
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

            f << setprecision(6) << tframe << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestamps[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }

        f.close();
        cout << endl << "trajectory saved!" << endl;

        // Edge-SLAM: split shutdown between client and server
        // Stop all threads
        SLAM.ClientShutdown();

        // Tracking time statistics
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
        for(int ni=0; ni<nImages; ni++)
        {
            totaltime+=vTimesTrack[ni];
        }
        cout << "-------" << endl << endl;
        cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
        cout << "mean tracking time: " << totaltime/nImages << endl;
    }
    else if(RunType.compare("server") == 0)
    {
        // Edge-SLAM
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        ORB_SLAM2::System SLAM(argv[1], argv[2], RunType, CommType, ORB_SLAM2::System::RGBD, true);

        // Edge-SLAM: split shutdown between client and server
        // Stop all threads
        SLAM.ServerShutdown();

        // Save camera trajectory
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }
    else
    {
        cerr << endl << "Client Usage: ./rgbd_tum VOC_PATH SETTINGS_PATH RUN_TYPE(client|server) COMMUNICATION_PROTOCOL(websocket|tcp) SEQUENCE_PATH ASSOCIATION_PATH" << endl;
        cerr << endl << "Server Usage: ./rgbd_tum VOC_PATH SETTINGS_PATH RUN_TYPE(client|server) COMMUNICATION_PROTOCOL(websocket|tcp)" << endl;
        return 1;
    }

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

