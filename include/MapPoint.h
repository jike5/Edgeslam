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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

// Edge-SLAM
#include "SerializeObject.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/set.hpp>

#include <opencv2/core/core.hpp>
#include <mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
    // Edge-SLAM
    friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & mnId & mnFirstKFid & mnFirstFrame & nObs;
            ar & lmMnId; ar & trSet; ar & lmSet;
            //ar & mTrackProjX & mTrackProjY & mTrackProjXR & mbTrackInView & mnTrackScaleLevel;
            //ar & mTrackViewCos;
            ar & mnTrackReferenceForFrame & mnLastFrameSeen;
            ar & mnBALocalForKF & mnFuseCandidateForKF;
            ar & mnLoopPointForKF & mnCorrectedByKF & mnCorrectedReference;
            //ar & mPosGBA;
            ar & mnBAGlobalForKF;
            ar & mWorldPos;
            ar & mObservations_id;
            ar & mNormalVector;
            ar & mDescriptor;
            //ar & mpRefKF;
            ar & mnVisible;
            ar & mnFound;
            ar & mbBad;
            ar & mpReplaced;
            ar & mfMinDistance;
            ar & mfMaxDistance;
        }

public:
    // Edge-SLAM
    MapPoint() {}

    // Edge-SLAM: added wchThread variable (1:tracking, 2:local-mapping)
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap, int wchThread);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF, int wchThread);

    // Edge-SLAM: check MapPoint fields and assign id. This function is either accesed from tracking thread on mobile-side or local-mapping thread on edge-side, thus no mutex needed
    void AssignId(bool isTracking);

    // Edge-SLAM: retrieve id based on working thread
    long unsigned int GetId();

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    // Edge-SLAM
    void SetReferenceKeyFrame(KeyFrame* ref);

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    // Edge-SLAM
    bool IsInKeyFrame(long int id);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    // Edge-SLAM: this function is either accesed from tracking thread on mobile-side or local-mapping thread on edge-side, thus no mutex needed
    void setMapPointer(Map* pMap){ mpMap = pMap;}

public:
    // Edge-SLAM: tracking thread id
    long unsigned int mnId;

    // Edge-SLAM
    bool trSet;     // Flag to indicate if tracking thread id is set or not
    bool lmSet;     // Flag to indicate if local-mapping thread id is set or not
    long unsigned int lmMnId;   // Local-mapping thread id

    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:
     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Edge-SLAM
     std::map<long int, size_t> mObservations_id;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;

     // Edge-SLAM: which thread is running (1:tracking, 2:local-mapping)
     int whichThread;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
