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
    /* !
     * @brief local mapping 구동시 main function이 되는 함수입니다.
     * @param None
     * @return void
    */
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    /* !
     * @brief mlNewKeyFrames에 데이터가 없애기 위해 사용하는 함수 (Current Key Frame에 mlNewKeyFrames의 첫번째 원소를 계속 대입)
     * @param None
     * @return void
    */
    void EmptyQueue();

    // Thread Synch

    /* !
     * @brief Stop이 필요할때 사용하는 함수
     * @param None
     * @return void
    */
    void RequestStop();
    
    /* !
     * @brief Local Mapper에 Reset을 요청하는 함수
     * @param None
     * @return void
    */
    void RequestReset();
    
    /* !
     * @brief Local Mapper에 Active Map의 Reset을 요청하는 함수
     * @param pMap (reset을 요청할 당시의 Active Current Map)
     * @return void
    */
    void RequestResetActiveMap(Map* pMap);

    /* !
     * @brief 간단하게 local mapping이 stop되는 시점에 실행되는 함수
     * @param None
     * @return Stop이 되었을 때 true, 그렇지 않을 때 false를 반환
    */
    bool Stop();

    /* !
     * @brief Run()을 끝내고 Map을 배포할 때 사용하는 함수
     * @param None
     * @return None
    */
    void Release();

    /* !
     * @brief local mapping이 stop 되어있는지 확인할때 사용하는 함수
     * @param None
     * @return Stop이면 true, 그렇지 않을 때 false를 반환
    */
    bool isStopped();

    /* !
     * @brief stop이 requested가 되었는지 확인할때 사용하는 함수
     * @param None
     * @return Stop이 요청되면 true, 그렇지 않을 때 false를 반환
    */
    bool stopRequested();

    /* !
     * @brief tracking.cc 에서 accept상태인지 아닌지 확인할때 사용하는 함수
     * @brief 실질적인 기능은 SetAcceptkeyframs에서 진행하고 마지막으로 true가 되면 해당 함수를 통해 tracking에서 받습니다. 
     * @param None
     * @return tracking.cc에서 accept하면 true, 그렇지 않으면 false를 반환
    */
    bool AcceptKeyFrames();

    /* !
     * @brief local mapping 종료시점을 알려줍니다. 해당 flag 값이 false되어있다면 tracking에서 데이터를 넘기지 않습니다.
     * @param flag값
     * @return None
    */
    void SetAcceptKeyFrames(bool flag);

    /* !
     * @brief tracking에서 createNewkeyframe진행시 local mapping이 멈춰져있는지 확인하는 작업
     * @param tracking에서 설정한 flag값
     * @return None
    */
    bool SetNotStop(bool flag);

    /* !
     * @brief AbortBA가 true가 되어야할때, 즉 keyframe을 받으면 안될때 실행되는 함수
     * @param None
     * @return None
    */
    void InterruptBA();

    /* !
     * @brief Local Mapper의 종료를 요청하는 함수
     * @param None
     * @return void
    */
    void RequestFinish();
    
    /* !
     * @brief Local Mapper가 종료되었는지 확인하는 함수
     * @param None
     * @return void
    */
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    /* !
     * @brief initializing 수행 여부확인
     * @param None
     * @return void
    */
    bool IsInitializing();

    /* !
     * @brief current keyframe 시간 확인, 없으면 0 대입
     * @param None
     * @return void
    */
    double GetCurrKFTime();

    /* !
     * @brief current keyframe 반환
     * @param None
     * @return Key Frame pointer
    */
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
    * @param None
    * @return void
    */
    void CreateNewMapPoints();

    /* !
     * @brief mlpRecentAddedMapPoints의 담겨있는 Map point의 상태를 확안 후, 조건에 맞지 않는 Map point를 걸러내기 위한 함수
     * @param None
     * @return void
    */
    void MapPointCulling();

    /* !
     * @brief neighbors keyframe을 검색하는 함수
     * @param None
     * @return void
    */
    void SearchInNeighbors();

    /* !
     * @brief 중복되는 Key Frame을 제거 및 골라내기 위한 함수
     * @param None
     * @return void
    */
    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);       // tracking.cc의 Compute12와 동일합니다. 
    cv::Matx33f ComputeF12_(KeyFrame* &pKF1, KeyFrame* &pKF2);  // tracking.cc의 Compute12_와 동일합니다. 

    /* !
    * @brief SkeySymmetricMatrix를 만들어 주는 함수 (위키백과 : https://ko.wikipedia.org/wiki/%EB%B0%98%EB%8C%80%EC%B9%AD_%ED%96%89%EB%A0%AC)
    * @param 3x1인 Matrix
    * @return 3x3 반대칭 행렬
    */
    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    /* !
    * @brief SkeySymmetricMatrix를 만들어 주는 함수 (위키백과 : https://ko.wikipedia.org/wiki/%EB%B0%98%EB%8C%80%EC%B9%AD_%ED%96%89%EB%A0%AC)
    * @param 3x1인 Matrix
    * @return 3x3 반대칭 행렬 (Output의 Type이 다름)
    */
    cv::Matx33f SkewSymmetricMatrix_(const cv::Matx31f &v);

    System *mpSystem;

    bool mbMonocular;
    bool mbInertial;

    /* !
     * @brief Reset이 요청되었을 경우, Reset을 실행하는 함수
     * @param None
     * @return void
    */
    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    /* !
     * @brief Local Mapper가 종료요청을 받았는지 확인하는 함수
     * @param None
     * @return void
    */
    bool CheckFinish();
    
    /* !
     * @brief Local Mapper의 종료를 set하는 함수
     * @param None
     * @return void
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
    
    /* !
    * @brief IMU가 초기화 되었을때, 10초 간격으로 스케일과 중력방향을 최적화합니다.
    * @param None
    * @return None
    */
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
