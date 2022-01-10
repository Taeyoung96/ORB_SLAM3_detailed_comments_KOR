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


#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Atlas.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "Config.h"

#include "KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;


class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;
    // aligned_allocator는 STL library와 Eigen을 같이 쓸 때 사용하는 연산자.
    //  map<KeyFrame*, g2o::Sim3>를 Line 51~52와 같이 쓴 것을 알 수 있다.
    // 출처 : https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html

public:

    LoopClosing(Atlas* pAtlas, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    /* !
    * @brief Tracking class를 Pointer로 설정해주기 위한 함수
    * @call system::System()
    * @param None
    * @return None
    */
    void SetTracker(Tracking* pTracker);

    /* !
    * @brief LocalMapping Class를 Pointer로 설정해주기 위한 함수
    * @call system::System()
    * @param None
    * @return None
    */
    void SetLocalMapper(LocalMapping* pLocalMapper);

    /* !
    * @brief Main function 
    * @param None
    * @return None
    */
    void Run();

    /* !
    * @brief LoopClosing::InsertKeyFrame()에 KeyFrame 전달
            LoopClosing detection을 위한 Queue에 CurrentKeyFrame을 추가
            LocalMapping::Run()함수를 확인해본 결과, LocalMapping에서 사용된 모든 Keyframe이 저장됨
    * @call LocalMapping::Run()
    * @param None
    * @return None
    */
    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();
    void RequestResetActiveMap(Map* pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    Viewer* mpViewer;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#ifdef REGISTER_TIMES
    double timeDetectBoW;

    std::vector<double> vTimeBoW_ms;
    std::vector<double> vTimeSE3_ms;
    std::vector<double> vTimePRTotal_ms;

    std::vector<double> vTimeLoopFusion_ms;
    std::vector<double> vTimeLoopEssent_ms;
    std::vector<double> vTimeLoopTotal_ms;

    std::vector<double> vTimeMergeFusion_ms;
    std::vector<double> vTimeMergeBA_ms;
    std::vector<double> vTimeMergeTotal_ms;

    std::vector<double> vTimeFullGBA_ms;
    std::vector<double> vTimeMapUpdate_ms;
    std::vector<double> vTimeGBATotal_ms;
#endif

protected:

    /* !
    * @brief mlpLoopKeyFrameQueue에 저장된 값의 유무를 반환
            LoopClosing::Run()에서 KeyFrame들에 대한 DB가 존재하면 LoopDetection을 시도하고, 그렇지 않으면 초기화를 해줌
    * @call  LoopClosing::Run()
    * @param None
    * @return !mlpLoopKeyFrameQueue.empty() // DB의 값이 존재하면 True, 존재하지 않으면 False
    */
    bool CheckNewKeyFrames();


    //Methods to implement the new place recognition algorithm
    /* !
    * @brief LoopClosure KF 및 Merged KF 을 탐색하는 것이 목적
            mlpLoopKeyFrameQueue의 DB값이 존재할 때 작동
    * @call  LoopClosing::Run()
    * @param None
    * @return boolean
    */
    bool NewDetectCommonRegions();

    /* !
    * @brief bool type 함수로서 간단하게 mappoint matching을 추출하고 설명하면 주어진 기준값 이상일때 
    * g2o값 ,즉 에러가 제거된 matrix를 뽑아내고 true를 반환한다.    
    * @param pCurrentKF 현재 Key Frame
    * @param pMatchedKF Matched된 Key Frame
    * @param gScw   world to camera Sim3 값
    * @param nNumProjMatches    Projection 시켰을 때 Matching된 point의 갯수
    * @param vpMPs  Map point를 담고 있는 vector (pointer로)
    * @param vpMatchedMPs   Matched된 Map point를 담고 있는 vector (pointer로)
    * @return 추출된 map point의 갯수가 nProjMatchesRep보다 클때 true, 그렇지 않으면 false
    */
    bool DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);

    bool DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, KeyFrame* &pMatchedKF, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                     int &nNumCoincidences, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);

    /* !
    * @brief 함수명 그대로 common regions를 last KF과 matched KF 사이에서 찾을수 있나의 확률적인 값. 즉 MatchedKF과 연관성이 있냐의 확률을 뜻함.
    * @param pCurrentKF 현재 Key Frame
    * @param pMatchedKF Matched된 Key Frame
    * @param gScw   world to camera Sim3 값
    * @param nNumProjMatches    Projection 시켰을 때 Matching된 point의 갯수
    * @param vpMPs  Map point를 담고 있는 vector (pointer로)
    * @param vpMatchedMPs   Matched된 Map point를 담고 있는 vector (pointer로)
    * @return boolean
    */
    bool DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);

    /* !
    * @brief MatchedKF과 CurrentKF에서 Matched MPs를 이용하여 유효한 MPs의 num을 추출하는 함수.
    * @param pCurrentKF 현재 Key Frame
    * @param pMatchedKFw 현재 Key Frame과 Covisibility graph로 연결된 Key Frame을 뽑아내기 위함 
    * @param g2oScw world to camera Sim3
    * @param spMatchedMPinOrigin 쓰이지 않음
    * @param vpMapPoints Map point를 담고 있는 vector (pointer로)
    * @param vpMatchedMapPoints Matched된 Map point를 담고 있는 vector (pointer로)
    * @return 유효한 Map point의 갯수
    */
    int FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
                                set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints,
                                vector<MapPoint*> &vpMatchedMapPoints);


    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint*> &vpMapPoints);
    void SearchAndFuse(const vector<KeyFrame*> &vConectedKFs, vector<MapPoint*> &vpMapPoints);

    /* !
    * @brief Loop를 Detect 했을 때, 찾을 Loop를 활용하여 Key Frame의 Sim3를 최적화하고, Map point도 최적화
    * @param None
    * @return None
    */
    void CorrectLoop();

    void MergeLocal();

    /* !
    * @brief Merge할 부분을 찾았을 때 IMU를 활용해서 Current Map에 있는 정보들(Key Frame, Map point, Essential Graph)을
    * @brief Merge Map에 있는 정보들과 합치고 최적화 
    * @param None
    * @return None
    */
    void MergeLocal2();

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpLastCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    //-------
    Map* mpLastMap;

    bool mbLoopDetected;
    int mnLoopNumCoincidences;
    int mnLoopNumNotFound;
    KeyFrame* mpLoopLastCurrentKF;
    g2o::Sim3 mg2oLoopSlw;
    g2o::Sim3 mg2oLoopScw;
    KeyFrame* mpLoopMatchedKF;
    std::vector<MapPoint*> mvpLoopMPs;
    std::vector<MapPoint*> mvpLoopMatchedMPs;
    bool mbMergeDetected;
    int mnMergeNumCoincidences;
    int mnMergeNumNotFound;
    KeyFrame* mpMergeLastCurrentKF;
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeSmw;
    g2o::Sim3 mg2oMergeScw;
    KeyFrame* mpMergeMatchedKF;
    std::vector<MapPoint*> mvpMergeMPs;
    std::vector<MapPoint*> mvpMergeMatchedMPs;
    std::vector<KeyFrame*> mvpMergeConnectedKFs;

    g2o::Sim3 mSold_new;
    //-------

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    bool mnFullBAIdx;



    vector<double> vdPR_CurrentTime;
    vector<double> vdPR_MatchedTime;
    vector<int> vnPR_TypeRecogn;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
