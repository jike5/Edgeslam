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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

// Edge-SLAM
#include "SerializeObject.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/set.hpp>

#include <mutex>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
// Edge-SLAM
private:
        friend class boost::serialization::access;
        //serialize LightKeyFrame class
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version){
                ar &  mnRelocQuery; ar &  mnRelocWords; ar &  mRelocScore;
                //ar &  mTcwGBA; ar &  mTcwBefGBA;
                ar &  mnBAGlobalForKF;
                ar &  mnId; ar &  mnFrameId; ar &  mTimeStamp; ar &  mnGridCols;
                ar &  mnGridRows; ar &  mfGridElementHeightInv; ar &  mfGridElementWidthInv;
                ar &  mnTrackReferenceForFrame;ar &  mnFuseTargetForKF; ar &  mnBALocalForKF;
                //ar &  mnBAFixedForKF; ar &  mnLoopQuery; ar &  mnLoopWords; ar &  mLoopScore;
                ar &  fx; ar &  fy; ar &  cx; ar & cy; ar & invfx; ar & invfy; ar & mbf;
                ar &  mb; ar & mThDepth; ar & N;
                ar &  mvKeys; ar &  mvKeysUn; ar & mvuRight;ar & mvDepth; ar & mDescriptors;
                ar &  mBowVec;
                ar &  mFeatVec;
                //ar &  mTcp;
                ar & mnScaleLevels;
                ar & mfScaleFactor; ar & mfLogScaleFactor; ar & mvScaleFactors;
                ar & mvLevelSigma2; ar &  mvInvLevelSigma2;
                ar & mnMinX; ar & mnMinY; ar & mnMaxX; ar & mnMaxY; ar & mK;
                ar & Tcw & Twc & Ow & Cw;
                ar & mvpMapPoints;
                ar & mGrid;
                //ar & mConnectedKeyFrameWeights;
                ar & mvpOrderedConnectedKeyFrames_ids;
                ar & mvOrderedWeights;
                ar & mbFirstConnection;
                ar & mpParent_id;
                ar & mspChildrens_ids;// & mspLoopEdges;
                ar & mbNotErase & mbToBeErased & mbBad;
                ar & mHalfBaseline;
                ar & mNeedNKF;
                ar & mPassedF;
                ar & mResetKF;
        };

public:
    // Edge-SLAM
    KeyFrame(){}

    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Edge-SLAM
    void ReconstructConnections();

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Edge-SLAM
    std::string GetParentId();
    std::set<long int> GetChildsIds();

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    // Edge-SLAM
    void setORBVocab(ORBVocabulary* pVoc){mpORBvocabulary = pVoc;}
    void setMapPointer(Map* pMap){mpMap = pMap;}
    void setKeyFrameDatabase(KeyFrameDatabase* KeyFrameDB){mpKeyFrameDB = KeyFrameDB;}

    // Edge-SLAM: mNeedNKF, mPassedF, mResetKF setter/getter functions
    void SetNeedNKF(int needNKF);
    void SetPassedF(bool passedF);
    void SetResetKF(bool resetKF);
    int GetNeedNKF();
    bool GetPassedF();
    bool GetResetKF();

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;

    // Edge-SLAM
    //const long unsigned int mnFrameId;
    long unsigned int mnFrameId;

    // Edge-SLAM
    //const double mTimeStamp;
    double mTimeStamp;

    // Edge-SLAM
    // Grid (to speed up feature matching)
    //const int mnGridCols;
    //const int mnGridRows;
    //const float mfGridElementWidthInv;
    //const float mfGridElementHeightInv;
    int mnGridCols;
    int mnGridRows;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Edge-SLAM
    // Calibration parameters
    //const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
    float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Edge-SLAM
    // Number of KeyPoints
    //const int N;
    int N;

    // Edge-SLAM
    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    //const std::vector<cv::KeyPoint> mvKeys;
    //const std::vector<cv::KeyPoint> mvKeysUn;
    //const std::vector<float> mvuRight; // negative value for monocular points
    //const std::vector<float> mvDepth; // negative value for monocular points
    //const cv::Mat mDescriptors;
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<float> mvuRight; // negative value for monocular points
    std::vector<float> mvDepth; // negative value for monocular points
    cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Edge-SLAM
    // Scale
    //const int mnScaleLevels;
    //const float mfScaleFactor;
    //const float mfLogScaleFactor;
    //const std::vector<float> mvScaleFactors;
    //const std::vector<float> mvLevelSigma2;
    //const std::vector<float> mvInvLevelSigma2;
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    // Edge-SLAM
    // Image bounds and calibration
    //const int mnMinX;
    //const int mnMinY;
    //const int mnMaxX;
    //const int mnMaxY;
    //const cv::Mat mK;
    int mnMinX;
    int mnMinY;
    int mnMaxX;
    int mnMaxY;
    cv::Mat mK;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Edge-SLAM
    std::vector<long int> mvpOrderedConnectedKeyFrames_ids;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Edge-SLAM
    std::set<long int> mspChildrens_ids;
    unsigned long int mpParent_id;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    // Edge-SLAM: these variables are either accesed from tracking thread on mobile-side or local-mapping thread on edge-side, thus no mutex needed
    int mNeedNKF;   // Flag to save the status of checking the keyframe selection conditions
    bool mPassedF;  // Flag to pass the status of one of the keyframe selection conditions to the edge
    bool mResetKF;  // Flag to pass reset signal to the edge

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
