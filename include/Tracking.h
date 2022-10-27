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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

// Edge-SLAM: imports for TCP
#include <TcpSocket.h>
#include "concurrentqueue.h"
#include "blockingconcurrentqueue.h"
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <thread>

#include <mutex>

namespace ORB_SLAM2
{
class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

typedef websocketpp::client<websocketpp::config::asio_client> client;

class connection_metadata {
public:
    typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

    connection_metadata(int id, websocketpp::connection_hdl hdl, std::string uri);

    void on_open(client * c, websocketpp::connection_hdl hdl);

    void on_fail(client * c, websocketpp::connection_hdl hdl);
    
    void on_close(client * c, websocketpp::connection_hdl hdl);

    void on_message_maps(moodycamel::ConcurrentQueue<std::string>* messageQueue, websocketpp::connection_hdl, client::message_ptr msg);

    void on_message_nothing(websocketpp::connection_hdl, client::message_ptr msg);

    websocketpp::connection_hdl get_hdl() const;
    
    int get_id() const;
    
    std::string get_status();

    void record_sent_message(std::string message);

private:
    int m_id;
    websocketpp::connection_hdl m_hdl;
    std::string m_status;
    std::string m_uri;
    std::string m_server;
    std::string m_error_reason;
    std::vector<std::string> m_messages;
};


class websocket_endpoint {
public:
    websocket_endpoint ();

    ~websocket_endpoint();

    // Used for KeyFrames and Maps connection, Maps from cloud need to be processed
    int connect(std::string const & uri, moodycamel::ConcurrentQueue<std::string>* messageQueue);
    // Used for Frames, there is no message from cloud need to be processed
    int connect(std::string const & uri);

    void close(int id, websocketpp::close::status::value code, std::string reason);

    // send KeyFrames/Frames to cloud, this function should run in an along thread
    int send(int id, moodycamel::BlockingConcurrentQueue<std::string>& messageQueue);

    connection_metadata::ptr get_metadata(int id) const;

private:
    typedef std::map<int,connection_metadata::ptr> con_list;

    client m_endpoint;
    websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;

    con_list m_connection_list;
    int m_next_id;
};

class Tracking
{

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
    
    // kubeedge: Add communication protocol parameters
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const string &strCommProtocol, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

    // Edge-SLAM
    void MUReset();

    // Edge-SLAM: TCP
    void static tcp_receive(moodycamel::ConcurrentQueue<std::string>* messageQueue, TcpSocket* socketObject, unsigned int maxQueueSize, std::string name);
    void static tcp_send(moodycamel::BlockingConcurrentQueue<std::string>* messageQueue, TcpSocket* socketObject, std::string name);
    
    // // Kubeedge: websocket
    // void static websocket_receive(moodycamel::ConcurrentQueue<std::string>* messageQueue, unsigned int maxQueueSize, std::string name, websocketpp::connection_hdl, client::message_ptr msg);
    // void static websocket_send(moodycamel::BlockingConcurrentQueue<std::string>* messageQueue, int id, std::string name);

    // Edge-SLAM: added destructor to destroy all connections on object termination
    ~Tracking();

protected:
    // Edge-SLAM: measure
    static std::chrono::high_resolution_clock::time_point msRelocLastMapUpdateStart;
    static std::chrono::high_resolution_clock::time_point msRelocLastMapUpdateStop;
    static std::chrono::high_resolution_clock::time_point msLastKeyFrameStart;
    static std::chrono::high_resolution_clock::time_point msLastKeyFrameStop;

    // Edge-SLAM: map update variables and functions
    void mapCallback(const std::string& msg);
    static vector<std::string> mapVec;          // Vector to hold local-map update
    static int mapCallbackCount;
    const static unsigned int TIME_KF;          // Set to: after how many ms from last created keyframe should a map update be accepted
    static unsigned int mnMapUpdateLastKFId;    // Set to: last keyframe id in map update
    static bool mapUpToDate;                    // Set to: true as long as map update is done and no new keyframes has been created in tracking
    const static unsigned int LOCAL_MAP_SIZE;   // Set to: number of keyframes in map after which the TIME_KF should be decreased
    static bool refKFSet;                       // ReferenceKF didn't change after map update
                                                // Set to: true to not skip next frame after update

    // Edge-SLAM: relocalization
    static bool msRelocStatus;
    const static int RELOC_FREQ;                // Set to: after how many ms from last received reloc map update, a new reloc frame should be sent

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    int NeedNewKeyFrame();
    void CreateNewKeyFrame(int needNKF);

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    // System
    System* mpSystem;

    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    // Edge-SLAM: thread variables
    std::thread* keyframe_thread ;
    std::thread* frame_thread ;
    std::thread* map_thread ;
    // Edge-SLAM: queue declarations
    moodycamel::BlockingConcurrentQueue<std::string> keyframe_queue;
    moodycamel::BlockingConcurrentQueue<std::string> frame_queue;
    moodycamel::ConcurrentQueue<std::string> map_queue;
    // Edge-SLAM: TcpSocket Objects
    TcpSocket* keyframe_socket;
    TcpSocket* frame_socket;
    TcpSocket* map_socket;

    // Kubeedge
    websocket_endpoint m_wsendpoint;
    int keyframes_id;
    int frames_id;
    int maps_id;
};


} //namespace ORB_SLAM

#endif // TRACKING_H
