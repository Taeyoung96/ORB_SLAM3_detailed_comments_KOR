/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Initializer.h"

#include <mutex>


namespace ORB_SLAM3
{

class System;
class Tracking;
class LoopClosing;
class Atlas;

class LocalMapping
{
public:
    LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName=std::string());

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Thread Synch
    void RequestStop();
    
    /* !
     * @brief  : Local Mapper에 Reset을 요청하는 함수
     * @param  : None
     * @return : void
    */
    void RequestReset();
    
    /* !
     * @brief  : Local Mapper에 Active Map의 Reset을 요청하는 함수
     * @param  : pMap (reset을 요청할 당시의 Active Current Map)
     * @return : void
    */
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    /* !
     * @brief  : Local Mapper의 종료를 요청하는 함수
     * @param  : None
     * @return : void
    */
    void RequestFinish();
    
    /* !
     * @brief  : Local Mapper가 종료되었는지 확인하는 함수
     * @param  : None
     * @return : void
    */
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

    std::mutex mMutexImuInit;

    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;
    double mInitTime;
    double mCostTime;
    bool mbNewInit;
    unsigned int mInitSect;
    unsigned int mIdxInit;
    unsigned int mnKFs;
    double mFirstTs;
    int mnMatchesInliers;

    bool mbNotBA1;
    bool mbNotBA2;
    bool mbBadImu;

    bool mbWriteStats;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;

#ifdef REGISTER_TIMES
    vector<double> vdKFInsert_ms;
    vector<double> vdMPCulling_ms;
    vector<double> vdMPCreation_ms;
    vector<double> vdLBA_ms;
    vector<double> vdKFCulling_ms;
    vector<double> vdLMTotal_ms;


    vector<double> vdLBASync_ms;
    vector<double> vdKFCullingSync_ms;
    vector<int> vnLBA_edges;
    vector<int> vnLBA_KFopt;
    vector<int> vnLBA_KFfixed;
    vector<int> vnLBA_MPs;
    int nLBA_exec;
    int nLBA_abort;
#endif
protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();

    /* !
    * @brief Current KeyFrame을 기준으로 인접한 Keyframe을 이용하여 triangulation을 수행하여 3D point를 생성하고
    *        Atlas-map과 current-map 객체에 3D point를 등록
    *        사용되는 위치: LocalMapping::Run();
    */
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();
    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);
    cv::Matx33f ComputeF12_(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);
    cv::Matx33f SkewSymmetricMatrix_(const cv::Matx31f &v);

    System *mpSystem;

    bool mbMonocular;
    bool mbInertial;

    /* !
     * @brief  : Reset이 요청되었을 경우, Reset을 실행하는 함수
     * @param  : None
     * @return : void
    */
    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    /* !
     * @brief  : Local Mapper가 종료요청을 받았는지 확인하는 함수
     * @param  : None
     * @return : void
    */
    bool CheckFinish();
    
    /* !
     * @brief  : Local Mapper의 종료를 set하는 함수
     * @param  : None
     * @return : void
    */
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;

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

    /* !
    * @brief IMU를 활용하여 Initialize를 수행, 카메라 포즈 및 카메라 속도 추정, Map point 최적화
    * @param priorG 이전 Gyro  
    * @param priorA 이전 Acc
    * @param bFIBA Full Inertial Bundle Adjustment에 대한 bool type
    * @return None
    */
    void InitializeIMU(float priorG = 1e2, float priorA = 1e6, bool bFirst = false);
    
    void ScaleRefinement();

    bool bInitializing;

    Eigen::MatrixXd infoInertial;
    int mNumLM;
    int mNumKFCulling;

    float mTinit;

    int countRefinement;

    //DEBUG
    ofstream f_lm;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
