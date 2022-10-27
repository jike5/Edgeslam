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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H
// Remove if you are using Boost Asio.
#define ASIO_STANDALONE

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>

#include <functional>
#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include <mutex>

// Edge-SLAM
#include "ORBVocabulary.h"
#include <unordered_set>
#include <stack>

// Edge-SLAM: TCP imports
#include <TcpSocket.h>
#include "concurrentqueue.h"
#include "blockingconcurrentqueue.h"
#include <thread>



namespace ORB_SLAM2
{
class Tracking;
class LoopClosing;
class Map;

typedef websocketpp::server<websocketpp::config::asio> server;

class utility_server {
public:
    utility_server(int port, moodycamel::ConcurrentQueue<std::string>* messageQueue, int numsQueue);
    utility_server(int port);
    ~utility_server();
    int send(moodycamel::BlockingConcurrentQueue<std::string>& messageQueue);

    void on_open(websocketpp::connection_hdl hdl);

    void message_handler(moodycamel::ConcurrentQueue<std::string>* messageQueue, websocketpp::connection_hdl hdl, server::message_ptr msg);

private:
    server m_endpoint;
    websocketpp::connection_hdl m_hdl;
    websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
};

class LocalMapping
{
public:
    // Edge-SLAM: added settings file path
    LocalMapping(Map* pMap, KeyFrameDatabase* pKFDB, ORBVocabulary* pVoc, const string &strSettingPath, const float bMonocular);
    
    // kubeedge: Add communication protocol parameters
    LocalMapping(Map* pMap, KeyFrameDatabase* pKFDB, ORBVocabulary* pVoc, const string &strSettingPath, const string &strCommProtocol, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    // Edge-SLAM
    void Reset();

    // Edge-SLAM: TCP
    void static tcp_receive(moodycamel::ConcurrentQueue<std::string>* messageQueue, TcpSocket* socketObject, unsigned int maxQueueSize, std::string name);
    void static tcp_send(moodycamel::BlockingConcurrentQueue<std::string>* messageQueue, TcpSocket* socketObject, std::string name);
    
    // Kubeedge: websocket
    void websocket_receive(moodycamel::ConcurrentQueue<std::string>* messageQueue, unsigned int maxQueueSize, websocketpp::connection_hdl, server::message_ptr msg);    
    void websocket_send(moodycamel::BlockingConcurrentQueue<std::string>* messageQueue, utility_server* server_ptr, std::string name);

    // Edge-SLAM: queue declarations
    moodycamel::ConcurrentQueue<std::string> keyframe_queue;
    moodycamel::ConcurrentQueue<std::string> frame_queue;
    moodycamel::BlockingConcurrentQueue<std::string> map_queue;

protected:
    // Edge-SLAM: measure
    static std::chrono::high_resolution_clock::time_point msLastMUStart;
    static std::chrono::high_resolution_clock::time_point msLastMUStop;

    // Edge-SLAM: map update
    void keyframeCallback(const std::string& msg);
    void frameCallback(const std::string& msg);
    bool NeedNewKeyFrame(KeyFrame* pKF);
    void sendLocalMapUpdate();
    const static int MAP_FREQ;  // Set to: after how many ms from last map update, a new map update should be sent
    const static int KF_NUM;    // Set to: how many keyframes should a map update consist of
    const static int CONN_KF;   // Set to: for every keyframe, how many connected keyframes should be included in a map update
    static bool msNewKFFlag;    // Flag used to only send map update when a new keyframe is received
    static stack<long unsigned int> msLatestKFsId;  // Stack to keep latest keyframes ids

    // Edge-SLAM
    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;
    // Similar to the one in Tracking.h
    unsigned int mnLastKeyFrameId;
    bool CheckReset();

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    // Edge-SLAM
    ORBVocabulary* mpORBVocabulary;

    Map* mpMap;

    // Edge-SLAM
    KeyFrameDatabase* mpKeyFrameDB;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    // Edge-SLAM: thread variables
    std::thread* keyframe_thread ;
    std::thread* frame_thread ;
    std::thread* map_thread ;

    // Edge-SLAM: TcpSocket Objects
    TcpSocket* keyframe_socket;
    TcpSocket* frame_socket;
    TcpSocket* map_socket;

    // Edge-SLAM: relocalization
    static vector<KeyFrame*> vpCandidateKFs;
    static unordered_set<long unsigned int> usCandidateKFsId;
    static bool msRelocStatus;      // Set to true when relocalization is happening
    static bool msRelocNewFFlag;    // Flag used to only send reloc map update when a new reloc frame is received
    const static int RELOC_FREQ;    // Set to: after how many ms from last reloc map, a new map should be sent
    void sendRelocMapUpdate();
    // Kubeedge
    utility_server* mp_keyframes_server;
    utility_server* mp_frames_server;
    utility_server* mp_maps_server;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
