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


#include "LoopClosing.h"

#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "ORBmatcher.h"
#include "G2oTypes.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM3
{

LoopClosing::LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbResetActiveMapRequested(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0), mnLoopNumCoincidences(0), mnMergeNumCoincidences(0),
    mbLoopDetected(false), mbMergeDetected(false), mnLoopNumNotFound(0), mnMergeNumNotFound(0)
{
    mnCovisibilityConsistencyTh = 3;
    mpLastCurrentKF = static_cast<KeyFrame*>(NULL);
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        //NEW LOOP AND MERGE DETECTION ALGORITHM
        //----------------------------
        if(CheckNewKeyFrames())
        {
            if(mpLastCurrentKF)
            {
                mpLastCurrentKF->mvpLoopCandKFs.clear();
                mpLastCurrentKF->mvpMergeCandKFs.clear();
            }
#ifdef REGISTER_TIMES
            timeDetectBoW = 0;
            std::chrono::steady_clock::time_point time_StartDetectBoW = std::chrono::steady_clock::now();
#endif
            bool bDetected = NewDetectCommonRegions();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndDetectBoW = std::chrono::steady_clock::now();
            double timeDetect = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndDetectBoW - time_StartDetectBoW).count();
            double timeDetectSE3 = timeDetect - timeDetectBoW;

            if(timeDetectBoW > 0)
            {
                vTimeBoW_ms.push_back(timeDetectBoW);
            }
            vTimeSE3_ms.push_back(timeDetectSE3);
            vTimePRTotal_ms.push_back(timeDetect);
#endif

            if(bDetected)
            {
                if(mbMergeDetected)
                {
                    if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO) &&
                        (!mpCurrentKF->GetMap()->isImuInitialized()))
                    {
                        cout << "IMU is not initilized, merge is aborted" << endl;
                    }
                    else
                    {
                        Verbose::PrintMess("*Merged detected", Verbose::VERBOSITY_QUIET);
                        Verbose::PrintMess("Number of KFs in the current map: " + to_string(mpCurrentKF->GetMap()->KeyFramesInMap()), Verbose::VERBOSITY_DEBUG);
                        cv::Mat mTmw = mpMergeMatchedKF->GetPose();
                        g2o::Sim3 gSmw2(Converter::toMatrix3d(mTmw.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTmw.rowRange(0, 3).col(3)),1.0);
                        cv::Mat mTcw = mpCurrentKF->GetPose();
                        g2o::Sim3 gScw1(Converter::toMatrix3d(mTcw.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTcw.rowRange(0, 3).col(3)),1.0);
                        g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();
                        g2o::Sim3 gSw1m = mg2oMergeSlw;

                        mSold_new = (gSw2c * gScw1);

                        if(mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial())
                        {
                            if(mSold_new.scale()<0.90||mSold_new.scale()>1.1){
                                mpMergeLastCurrentKF->SetErase();
                                mpMergeMatchedKF->SetErase();
                                mnMergeNumCoincidences = 0;
                                mvpMergeMatchedMPs.clear();
                                mvpMergeMPs.clear();
                                mnMergeNumNotFound = 0;
                                mbMergeDetected = false;
                                Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
                                continue;
                            }
                            // If inertial, force only yaw
                            if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO) &&
                                   mpCurrentKF->GetMap()->GetIniertialBA1()) // TODO, maybe with GetIniertialBA1
                            {
                                Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                                phi(0)=0;
                                phi(1)=0;
                                mSold_new = g2o::Sim3(ExpSO3(phi),mSold_new.translation(),1.0);
                            }
                        }

                        mg2oMergeSmw = gSmw2 * gSw2c * gScw1;

                        mg2oMergeScw = mg2oMergeSlw;

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_StartMerge = std::chrono::steady_clock::now();
#endif
                        if (mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO)
                            MergeLocal2();
                        else
                            MergeLocal();
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndMerge = std::chrono::steady_clock::now();
                        double timeMerge = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMerge - time_StartMerge).count();
                        vTimeMergeTotal_ms.push_back(timeMerge);
#endif
                    }

                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(1);

                    // Reset all variables
                    mpMergeLastCurrentKF->SetErase();
                    mpMergeMatchedKF->SetErase();
                    mnMergeNumCoincidences = 0;
                    mvpMergeMatchedMPs.clear();
                    mvpMergeMPs.clear();
                    mnMergeNumNotFound = 0;
                    mbMergeDetected = false;

                    if(mbLoopDetected)
                    {
                        // Reset Loop variables
                        mpLoopLastCurrentKF->SetErase();
                        mpLoopMatchedKF->SetErase();
                        mnLoopNumCoincidences = 0;
                        mvpLoopMatchedMPs.clear();
                        mvpLoopMPs.clear();
                        mnLoopNumNotFound = 0;
                        mbLoopDetected = false;
                    }

                }

                if(mbLoopDetected)
                {
                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(0);


                    Verbose::PrintMess("*Loop detected", Verbose::VERBOSITY_QUIET);

                    mg2oLoopScw = mg2oLoopSlw;
                    if(mpCurrentKF->GetMap()->IsInertial())
                    {
                        cv::Mat Twc = mpCurrentKF->GetPoseInverse();
                        g2o::Sim3 g2oTwc(Converter::toMatrix3d(Twc.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(Twc.rowRange(0, 3).col(3)),1.0);
                        g2o::Sim3 g2oSww_new = g2oTwc*mg2oLoopScw;

                        Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());

                        if (fabs(phi(0))<0.008f && fabs(phi(1))<0.008f && fabs(phi(2))<0.349f)
                        {
                            if(mpCurrentKF->GetMap()->IsInertial())
                            {
                                // If inertial, force only yaw
                                if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO) &&
                                        mpCurrentKF->GetMap()->GetIniertialBA2())
                                {
                                    phi(0)=0;
                                    phi(1)=0;
                                    g2oSww_new = g2o::Sim3(ExpSO3(phi),g2oSww_new.translation(),1.0);
                                    mg2oLoopScw = g2oTwc.inverse()*g2oSww_new;
                                }
                            }

                            mvpLoopMapPoints = mvpLoopMPs;//*mvvpLoopMapPoints[nCurrentIndex];

#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_StartLoop = std::chrono::steady_clock::now();
#endif
                            CorrectLoop();
#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_EndLoop = std::chrono::steady_clock::now();
                            double timeLoop = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLoop - time_StartLoop).count();
                            vTimeLoopTotal_ms.push_back(timeLoop);
#endif
                        }
                        else
                        {
                            cout << "BAD LOOP!!!" << endl;
                        }
                    }
                    else
                    {
                        mvpLoopMapPoints = mvpLoopMPs;
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_StartLoop = std::chrono::steady_clock::now();
#endif
                        CorrectLoop();

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndLoop = std::chrono::steady_clock::now();
                        double timeLoop = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLoop - time_StartLoop).count();
                        vTimeLoopTotal_ms.push_back(timeLoop);
#endif
                    }

                    // Reset all variables
                    mpLoopLastCurrentKF->SetErase();
                    mpLoopMatchedKF->SetErase();
                    mnLoopNumCoincidences = 0;
                    mvpLoopMatchedMPs.clear();
                    mvpLoopMPs.clear();
                    mnLoopNumNotFound = 0;
                    mbLoopDetected = false;
                }

            }
            mpLastCurrentKF = mpCurrentKF;
        }

        ResetIfRequested();

        if(CheckFinish()){
            break;
        }

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF); // LoopClosing detection을 위한 Queue에 CurrentKeyFrame을 추가
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty()); // DB의 값이 존재하면 True, 존재하지 않으면 False
} 

bool LoopClosing::NewDetectCommonRegions()
{
    // VI. Map Merging And Loop Closing (In ORB-SLAM3 paper)
    // - A. PLACE RECOGNITION
    // 1 . DBOW2 candidate keyframes                   : Atals DBoW2 DB를 이용하여 가장 유사한 3개의 KF를 검색
    // 2 . Local window                                : 검색된 KF과 Currnet_KF으로부터 keyPoint 및 MapPoint에 대한 local-window를 정의(2D-2D matching/3D-3D matching)
    // 3 . 3D aligining transformation                 : Ransac을 이용하여 KF와 검색된 유사한 KF간의 Transforamtion matrix를 추정
    // 4 . Guided matching refinement                  : 추정된 Transformation matrix를 이용하여 matching point를 추가로 검색하고,
    //                                                   더 작은 window를 이용하여 non-linear optimization으로 tansforamtion matrix를 추정 및 미세조정
    // 5 . Verification in three covisible keyframes   : place recognition 지연 및 누락을 피하기 위해 추가 검증을 수행(연속된 3개의 KF의 T를 확인 및 유효성 검사)
    // 6 . VI Gravity direction verification           : 추정된 T로부터 pitch / roll / yaw 각도가 임계값 미만인지 검사 (정확히 이함수에서 하는지 미확인)

    // 코드 흐름상 2가지 케이스로 종료가 될 수 있음    
    // a) 1->2->3 or 4
    // b) 1->2->3 or 4 (3 or 4에서 실패하면) -> 5
    
    // ===========================================================================================================================================================================
    // 초기 설정부분
    {
        unique_lock<mutex> lock(mMutexLoopQueue);   // LoopDetection을 위해서 새로운 Keyframe이 mlpLoopKeyFrameQueue에 추가되지 않도록 lock을 걸음
        mpCurrentKF = mlpLoopKeyFrameQueue.front(); // 가장 최근의 KF를 가져옴
        mlpLoopKeyFrameQueue.pop_front();           // mlpLoopKeyFrameQueue 가장 최근의 KF를 Queue에서 제거
        
        // Avoid that a keyframe can be erased while it is being process by this thread
        // 다른 모듈에서 KF를 제거하지 못하도록 방지
        // 실질적으로 KeyFrame::mbNotErase=true가 되면서 KeyFrame::SetErase() 함수가 작동하지 않도록 방지
        // KeyFrame::SetBadFlag()은 정확히 어디서 사용된다고 말하기 어려우나, LoopClosing.cc에서 매우 많이 사용됨(KeyFrame::SetErase())
        mpCurrentKF->SetNotErase(); 
        
        mpCurrentKF->mbCurrentPlaceRecognition = true; // 일단 사용되지 않음 (전체 검색을 했으나 사용되는 위치가 없음)
                                                       // 의미 자체는 current_KF은 PlaceRecognition을 아직 수행하지 않았지만 일단 성공 되었다고 보고 true로 설정
                                                       // 후반부 return할때 PlaceRecognition이 실패하면 false로 변경되고 반환됨

        mpLastMap = mpCurrentKF->GetMap(); // Map class로부터 현재 map정보의 pointer를 가져옴
    }
    
    // mpLastMap->IsInertial(): IMU 사용 유무 및 IMU 초기화 유무
    // mpLastMap->GetIniertialBA1(): LoopClosing::Run()내의 LoopClosing::MergeLocal2()함수에서 Map에대한 IMU 초기화를 해줌
    if(mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA1())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }
    
    // 스테레오 카메라 사용 유무
    // KeyFrame의 개수가 5개 미만인경우
    if(mpTracker->mSensor == System::STEREO && mpLastMap->GetAllKeyFrames().size() < 5) //12
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // Keyframe의 개수
    // KeyFrame의 개수가 12개 미만인경우
    if(mpLastMap->GetAllKeyFrames().size() < 12)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // ===========================================================================================================================================================================
    // (In papaer, 5.A.3~4)이 해당 (3D aligning transformation, Guided matching refinement)

    //Check the last candidates with geometric validation, 기하학 검증으로 마지막 후보 확인
    // merge map <-> Loop closing
    // map merging => 성공적인 Place recognition일 때, Keyframe Ka와 Active map 요소 Ma, 
    //                그리고 Km과 Stored map 요소인 Mm 사이 data association
    //     Two step 1) Merge는 공시성 그래프의 Ka들과 Km에 의해 정의된 welding window에서 수행됨
    //              2) 이 수정은 포즈 그래프 최적화에 의해 병합된 맵의 나머지 부분으로 전파된다.
    // Loop closing => map merging과 유사하지만, 두 키프레임 모두 Active map에 속해 있음.

    // Loop candidates
    bool bLoopDetectedInKF = false; // 추후 Loop detcted frame이 발견되면 true
    bool bCheckSpatial = false;     // 사용되지 않음
    
    // mnMergeNumCoincidences는 현재 KF와 이전 KF들로부터 공통된 영역을 발견한 횟수를 의미
    // mnMergeNumCoincidences=0일 경우 아래의 DetectCommonRegionsFromBoW()를 통해 mnMergeNumCoincidences의 개수가 정해짐  (In papaer, 5.A.2) Local Window
    // mnMergeNumCoincidences>0일 경우 아래의 if()문에서 증가하거나 DetectCommonRegionsFromBoW()를 통해 개수가 변동됨      (In papaer, 5.A.2) Local Window
    // DetectCommonRegionsFromBoW()는 이전 프레임들에서 공통뷰를 찾는데 사용되는 함수, 연속된 3개 이상의 KF가 발견되면 mbLoopDetected=True
    if(mnLoopNumCoincidences > 0)
    {
        bCheckSpatial = true;
        // Find from the last KF candidates 
        // 1. DBoW2 candidate keyframes, Atlas DB에 active keyframe Ka를 입력해야함.
        // mpCurrentKF = mlpLoopKeyFrameQueue.front()
        // 초기 mpLoopLastCurrentKF DetectCommonRegionsFromBoW()에서 받고, 이후에는 mpLoopLastCurrentKF = mpCurrentKF 이전 값을 받는다.
        cv::Mat mTcl = mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse();
        
        // Map 병합 이전에는 SE3로 변환행렬을 구하는게 아닌 Sim3로 구함.
        g2o::Sim3 gScl(Converter::toMatrix3d(mTcl.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTcl.rowRange(0, 3).col(3)),1.0);
        g2o::Sim3 gScw = gScl * mg2oLoopSlw;

        // 계산된 3차원 닮음 변환 관계를 바탕으로 지도 정렬과 지도 병합을 수행한다. SLAM B 지도의 요소들(프레임, 3차원 점, 3차원 선)에 계산된 3차원 닮음 변환이 적용되어
        // SLAM A 지도에서 중첩이 발생한 프레임을 기준으로 정렬된다. 이때, SLAM A의 중첩된 프레임에서 발견되는 3차원 점과 3차원 선이 SLAM B의 현재 프레임에 투영된 후 
        //매칭쌍을 탐색해 겹치는 점과 선을 찾아낸다. 정렬된 SLAM B 지도에서 겹치는 점과 선을 제외하고 그 외의 프레임, 3차원 점, 3차원선을 SLAM A에 추가하여 병합 된 지도
        // 를 완성한다.
        int numProjMatches = 0;               // DetectAndReffineSim3FromLastKF()내의 FindMatchesByProjection()함수에서 정해짐

        // 변환행렬에 대한 각 가설을 찾기 위해 3d-3d 일치의 최소 집합을 사용하는 Horn 알고리즘을 사용
        vector<MapPoint*> vpMatchedMPs;      // vpMatchedMPs = vpBestMatchedMapPoints,  map point와 일치하는 점 숫자
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches, mvpLoopMPs, vpMatchedMPs);

        // mvpLoopMPs = best Map points, vpMatchedMPs = best Matched Map points
        // RANSAC을 활용해서 Km Local window의 map point를 Ka의 것과 더 잘 정렬하는 변환 행렬 활용하기 위함
        // => Inlier 찾기...
        if(bCommonRegion)
        {
            bLoopDetectedInKF = true;          // Loop Detected 인지 true값으로 변환

            mnLoopNumCoincidences++;           // ++
            mpLoopLastCurrentKF->SetErase();   // SetBadflag
            mpLoopLastCurrentKF = mpCurrentKF; // Last Key Frame에 입력
            mg2oLoopSlw = gScw;                // 변환행렬 재활용
            mvpLoopMatchedMPs = vpMatchedMPs;  // Update matched map points and replace if duplicated, CorrectLoop()에서 활용됨.


            mbLoopDetected = mnLoopNumCoincidences >= 3;  // 3개 이상 일 때 Loop Detected (True)
            mnLoopNumNotFound = 0;

            if(!mbLoopDetected)
            {
                cout << "PR: Loop detected with Reffine Sim3" << endl;
            }
        }
        else
        {
            bLoopDetectedInKF = false;

            mnLoopNumNotFound++;          // 찾지 못한건지 확인
            if(mnLoopNumNotFound >= 2)    // 찾지 못한 것이면 다시 Clear
            {
                mpLoopLastCurrentKF->SetErase();
                mpLoopMatchedKF->SetErase();
                mnLoopNumCoincidences = 0;
                mvpLoopMatchedMPs.clear();
                mvpLoopMPs.clear();
                mnLoopNumNotFound = 0;
            }
        }
    }

    //Merge candidates
    bool bMergeDetectedInKF = false;

    // (In papaer, 5.A.3~4)이 해당 (3D aligning transformation, Guided matching refinement)
    // mnMergeNumCoincidences는 현재 KF와 이전 KF들로부터 공통된 영역을 발견한 횟수를 의미
    // mnMergeNumCoincidences=0일 경우 아래의 DetectCommonRegionsFromBoW()를 통해 mnMergeNumCoincidences의 개수가 정해짐
    // mnMergeNumCoincidences>0일 경우 아래의 if()문에서 증가하거나 DetectCommonRegionsFromBoW()를 통해 개수가 변동됨
    // DetectCommonRegionsFromBoW()는 이전 프레임들에서 공통뷰를 찾는데 사용되는 함수, 연속된 3개 이상의 KF가 발견되면 mbLoopDetected=True
    if(mnMergeNumCoincidences > 0)
    {
        // Find from the last KF candidates
        cv::Mat mTcl = mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse();
        g2o::Sim3 gScl(Converter::toMatrix3d(mTcl.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTcl.rowRange(0, 3).col(3)),1.0);
        g2o::Sim3 gScw = gScl * mg2oMergeSlw;
        int numProjMatches = 0;
        vector<MapPoint*> vpMatchedMPs;

        // DetectAndReffineSim3FromLastKF()는 CurrentKF와 잠재적 LoopKF영역에 대하여 공통뷰가 존재하는 Transformation & ProjectionMatching을 통해 검사
        // ProjectionMatching point의 개수가 일정개수 이상이면 True를 반환
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches, mvpMergeMPs, vpMatchedMPs);
        if(bCommonRegion)
        {
            bMergeDetectedInKF = true; // Merge할 KF이 존재한다고 설정

            mnMergeNumCoincidences++;  // CurrentKF와 매칭되는 view의 개수를 증가
            mpMergeLastCurrentKF->SetErase(); // 
            mpMergeLastCurrentKF = mpCurrentKF;
            mg2oMergeSlw = gScw;
            mvpMergeMatchedMPs = vpMatchedMPs;

            mbMergeDetected = mnMergeNumCoincidences >= 3; // CurrentKF와 매칭되는 view의 개수가 3개이상일 경우 True
        }
        else
        {
            mbMergeDetected = false;
            bMergeDetectedInKF = false;

            mnMergeNumNotFound++;
            if(mnMergeNumNotFound >= 2)
            {
                mpMergeLastCurrentKF->SetErase();
                mpMergeMatchedKF->SetErase();
                mnMergeNumCoincidences = 0;
                mvpMergeMatchedMPs.clear();
                mvpMergeMPs.clear();
                mnMergeNumNotFound = 0;
            }


        }
    }

    // mbMergeDetected: 공통뷰가 이미 발견됬을 경우, 위의 if()문을 
    // mbLoopDetected : DetectCommonRegionsFromBoW()에서 DBoW로 Current_KF와 이전 KF의 공통뷰가 발견됬을 경우 True
    if(mbMergeDetected || mbLoopDetected)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        return true;
    }


    // ===========================================================================================================================================================================
    // (In papaer, 5.A.1) DBoW2 candiate keyframe에 해당
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames(); // Current_KF과 연결된 covisible_KF의 포인터를 가져옴
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;                              // Current_KF의 DBoW2 vector의 포인터를 가져옴

    // Extract candidates from the bag of words
    // LoopDetectedKF과 MergedKF에 대한 후보군을 저장할 변수 선언
    // vpLoopBowCand: ActiveMap에 대한 KF 후보군
    // vpMergeBowCand: StoredMap에 대한 KF 후보군
    vector<KeyFrame*> vpMergeBowCand, vpLoopBowCand;

    // 위의 5.A.3과 5.A.4에서 bMergeDetectedInKF와 bLoopDetectedInKF를 감지하지 못한 경우 == True
    // bMergeDetectedInKF : Merge KF를 감지 못함
    // bLoopDetectedInKF  : Loop KF를 감지 못함
    if(!bMergeDetectedInKF || !bLoopDetectedInKF)
    {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartDetectBoW = std::chrono::steady_clock::now();
#endif
        // (In papaer, 5.A.1) DBoW2 candiate keyframe에 해당
        // Search in BoW        
        // Current_KF와 공통뷰를 갖는 KF중에서 가장 최적의 KF들중 3개를 선택
        // Active KF을 사용하여 Atals DBoW2 DB를 query하여 Ka(Keyframe_active)와 함께 볼 수 있는 KF을 제외하고 가장 유사한 3개의 KF를 검색
        // Place recognition을 위한 후보군을 Km이라고 논문에서 정의됨
        // @param mpCurrentKF: CurrentKF
        // @param vpLoopBowCand: ActiveMap에 대한 KF 후보군
        // @param vpMergeBowCand: StoredMap에 대한 KF 후보군
        mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand,3);
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndDetectBoW = std::chrono::steady_clock::now();
        timeDetectBoW = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndDetectBoW - time_StartDetectBoW).count();
#endif
    }


    // ===========================================================================================================================================================================
    // (In papaer, 5.A.2) Local Window에 해당
    // (In papaer, 5.A.5) Verification in three covisible keyframes에 해당
    // bLoopDetectedInKF : Loop KF를 감지 못함
    // !vpLoopBowCand.empty() : LoopKF을 위한 후보군이 검색된 경우
    if(!bLoopDetectedInKF && !vpLoopBowCand.empty())
    {   
        // Place recognition을 위한 후보군 Km에 대해 공통영역이 가장 잘 검출되는 KF와 이에 해당하는 Map-Point에 대한 Local-widnow를 정의
        // DBoW2를 이용한 Ka의 Keypoint와 Km에 대한 local-window간의 2D-2D matching / 3D-3D matching을 사용하여 공통뷰의 개수를 업데이트        
        // @brief DetectCommonRegionsFromBoW()는 이전 프레임들에서 공통뷰를 찾는데 사용되는 함수
        // @Param mnLoopNumCoincidences는 CurrentKF와 공통되는 뷰의 개수를 의미하는데, 함수인자를 포인터로 넘겨주어 공통뷰의 개수를 업데이트
        // @return 연속된 3개 이상의 KF가 발견되면 mbLoopDetected=True
        mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs);
    }
    
    
    // (In papaer, 5.A.2) Local Window에 해당
    // (In papaer, 5.A.5) Verification in three covisible keyframes에 해당
    // bMergeDetectedInKF : Merge KF를 감지 못함
    // !vpMergeBowCand.empty() : MergeKF을 위한 후보군이 검색된 경우
    if(!bMergeDetectedInKF && !vpMergeBowCand.empty())
    {
        // 위와 동일        
        // 후보군 Km에 대해 공통영역이 가장 잘 검출되는 KF와 이에 해당하는 Map-Point에 대한 Local-widnow를 정의
        // DBoW2를 이용한 Ka의 Keypoint와 Km에 대한 local-window간의 2D-2D matching / 3D-3D matching을 사용하여 공통뷰의 개수를 업데이트        
        // @brief DetectCommonRegionsFromBoW()는 이전 프레임들에서 공통뷰를 찾는데 사용되는 함수
        // @Param mnLoopNumCoincidences는 CurrentKF와 공통되는 뷰의 개수를 의미하는데, 함수인자를 포인터로 넘겨주어 공통뷰의 개수를 업데이트
        // @return 연속된 3개 이상의 KF가 발견되면 mbLoopDetected=True
        mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
    }

    
    mpKeyFrameDB->add(mpCurrentKF); // CurrentKF에서 Bag-of-word를 추출하고 DB에 저장

    // Merged KF 또는 LoopDetected KF을 발견한 경우 True를 반환
    if(mbMergeDetected || mbLoopDetected)
    {
        return true;
    }

    // Merged KF 또는 LoopDetected KF이 아닌경우, Queue에서 제거하고 False를 반환
    mpCurrentKF->SetErase();
    mpCurrentKF->mbCurrentPlaceRecognition = false; // 일단 사용되지 않음 (전체 검색을 했으나 사용되는 위치가 없음)
                                                    // 의미 자체는 current_KF은 PlaceRecognition을 아직 수행하지 않았지만 일단 성공 되었다고 보고 true로 설정
                                                    // 후반부 return할때 PlaceRecognition이 실패하면 false로 변경되고 반환됨

    return false;
}

bool LoopClosing::DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                 std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
{
   set<MapPoint*> spAlreadyMatchedMPs; //container 선언
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs); 
    //best covisiblity를 가진 keyframe과 현재 covisibility keyframe을 비교하여 둘과 matching이 되는 keyframe을 저정한뒤
    //해당 keyframe에서 mappoint를 추출하고 orb matcher 0.9 기준으로 걸러내고 
    //이후 걸러진 mappoint들을 searchbyprojection을 통해 (by hamming distance) 최종 num을 뽑아냄.


    //아래의 int값들은 threshold
    int nProjMatches = 30; 
    int nProjOptMatches = 50;
    int nProjMatchesRep = 100;

    //위에서 추출한 num값인 nNumProjMatches가 기준값 30이상일때 실행
    if(nNumProjMatches >= nProjMatches)
    {
        cv::Mat mScw = Converter::toCvMat(gScw); //함수 parameter gScw를 convert --> toCvSE3(s*eigR,eigt);
        cv::Mat mTwm = pMatchedKF->GetPoseInverse(); //함수 parameter pMatchedKF에서 GetPoseInverse matrix 추출
        g2o::Sim3 gSwm(Converter::toMatrix3d(mTwm.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTwm.rowRange(0, 3).col(3)),1.0); //g20의 matrix 3x3 포함하고있는 Sim3로 선언 (3개 param 구성)
        g2o::Sim3 gScm = gScw * gSwm; //위의 4x4 matrix 연산
        Eigen::Matrix<double, 7, 7> mHessian7x7; //Hessian matrix

        bool bFixedScale = mbFixScale;       // TODO CHECK; Solo para el monocular inertial , 단순 선언
        if(mpTracker->mSensor==System::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetIniertialBA2()) //monocular 거나 iniertialBA2 안됬을때
            bFixedScale=false; //false 
        int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);
        //int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale, Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints)
        //간단히 말하면 mpCurrentKF, pMatchedKF에서 매칭된 MPs와 vpMatchedMPs를 조합했을때 gScm으로 estimate를 진행한다. matching이 된 MPs들로 Hessian matrix를 통해 optimize를 진행후 MPs중 inliner에 해당하는 갯수를 추출


        if(numOptMatches > nProjOptMatches) //위에서 추출한 int값이 nProjOptMatches보다 크면 진행
        {
            g2o::Sim3 gScw_estimation(Converter::toMatrix3d(mScw.rowRange(0, 3).colRange(0, 3)),
                           Converter::toVector3d(mScw.rowRange(0, 3).col(3)),1.0); //Sim3 선언

            vector<MapPoint*> vpMatchedMP; //vecter 선언
            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));  //vecter resize

            nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs); //다시한번 Find matches (vpMatchedMP 변경)
            if(nNumProjMatches >= nProjMatchesRep) //추출된 map point의 갯수가 nProjMatchesRep보다 클때 성공
            {
                gScw = gScw_estimation;
                return true;
            }
        }
    }
    return false;
}

bool LoopClosing::DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, KeyFrame* &pMatchedKF2, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                             int &nNumCoincidences, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
{
    int nBoWMatches = 20;
    int nBoWInliers = 15;
    int nSim3Inliers = 20;
    int nProjMatches = 50;
    int nProjOptMatches = 80;

    set<KeyFrame*> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

    int nNumCovisibles = 5;

    ORBmatcher matcherBoW(0.9, true);
    ORBmatcher matcher(0.75, true);
    int nNumGuidedMatching = 0;

    KeyFrame* pBestMatchedKF;
    int nBestMatchesReproj = 0;
    int nBestNumCoindicendes = 0;
    g2o::Sim3 g2oBestScw;
    std::vector<MapPoint*> vpBestMapPoints;
    std::vector<MapPoint*> vpBestMatchedMapPoints;

    int numCandidates = vpBowCand.size();
    vector<int> vnStage(numCandidates, 0);
    vector<int> vnMatchesStage(numCandidates, 0);

    int index = 0;
    for(KeyFrame* pKFi : vpBowCand)
    {
        if(!pKFi || pKFi->isBad())
            continue;


        // Current KF against KF with covisibles version
        std::vector<KeyFrame*> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
        vpCovKFi.push_back(vpCovKFi[0]);
        vpCovKFi[0] = pKFi;

        std::vector<std::vector<MapPoint*> > vvpMatchedMPs;
        vvpMatchedMPs.resize(vpCovKFi.size());
        std::set<MapPoint*> spMatchedMPi;
        int numBoWMatches = 0;

        KeyFrame* pMostBoWMatchesKF = pKFi;
        int nMostBoWNumMatches = 0;

        std::vector<MapPoint*> vpMatchedPoints = std::vector<MapPoint*>(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
        std::vector<KeyFrame*> vpKeyFrameMatchedMP = std::vector<KeyFrame*>(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));

        int nIndexMostBoWMatchesKF=0;
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(!vpCovKFi[j] || vpCovKFi[j]->isBad())
                continue;

            int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
            if (num > nMostBoWNumMatches)
            {
                nMostBoWNumMatches = num;
                nIndexMostBoWMatchesKF = j;
            }
        }

        bool bAbortByNearKF = false;
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
            {
                bAbortByNearKF = true;
                break;
            }

            for(int k=0; k < vvpMatchedMPs[j].size(); ++k)
            {
                MapPoint* pMPi_j = vvpMatchedMPs[j][k];
                if(!pMPi_j || pMPi_j->isBad())
                    continue;

                if(spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
                {
                    spMatchedMPi.insert(pMPi_j);
                    numBoWMatches++;

                    vpMatchedPoints[k]= pMPi_j;
                    vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                }
            }
        }

        if(!bAbortByNearKF && numBoWMatches >= nBoWMatches) // TODO pick a good threshold
        {
            // Geometric validation

            bool bFixedScale = mbFixScale;
            if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                bFixedScale=false;

            Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
            solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers

            bool bNoMore = false;
            vector<bool> vbInliers;
            int nInliers;
            bool bConverge = false;
            cv::Mat mTcm;
            while(!bConverge && !bNoMore)
            {
                mTcm = solver.iterate(20,bNoMore, vbInliers, nInliers, bConverge);
            }

            if(bConverge)
            {
                vpCovKFi.clear();
                vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                int nInitialCov = vpCovKFi.size();
                vpCovKFi.push_back(pMostBoWMatchesKF);
                set<KeyFrame*> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

                set<MapPoint*> spMapPoints;
                vector<MapPoint*> vpMapPoints;
                vector<KeyFrame*> vpKeyFrames;
                for(KeyFrame* pCovKFi : vpCovKFi)
                {
                    for(MapPoint* pCovMPij : pCovKFi->GetMapPointMatches())
                    {
                        if(!pCovMPij || pCovMPij->isBad())
                            continue;

                        if(spMapPoints.find(pCovMPij) == spMapPoints.end())
                        {
                            spMapPoints.insert(pCovMPij);
                            vpMapPoints.push_back(pCovMPij);
                            vpKeyFrames.push_back(pCovKFi);
                        }
                    }
                }

                g2o::Sim3 gScm(Converter::toMatrix3d(solver.GetEstimatedRotation()),Converter::toVector3d(solver.GetEstimatedTranslation()),solver.GetEstimatedScale());
                g2o::Sim3 gSmw(Converter::toMatrix3d(pMostBoWMatchesKF->GetRotation()),Converter::toVector3d(pMostBoWMatchesKF->GetTranslation()),1.0);
                g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                cv::Mat mScw = Converter::toCvMat(gScw);


                vector<MapPoint*> vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
                vector<KeyFrame*> vpMatchedKF;
                vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));
                int numProjMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);

                if(numProjMatches >= nProjMatches)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> mHessian7x7;

                    bool bFixedScale = mbFixScale;
                    if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                        bFixedScale=false;

                    int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, mbFixScale, mHessian7x7, true);

                    if(numOptMatches >= nSim3Inliers)
                    {
                        g2o::Sim3 gSmw(Converter::toMatrix3d(pMostBoWMatchesKF->GetRotation()),Converter::toVector3d(pMostBoWMatchesKF->GetTranslation()),1.0);
                        g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                        cv::Mat mScw = Converter::toCvMat(gScw);

                        vector<MapPoint*> vpMatchedMP;
                        vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
                        int numProjOptMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

                        if(numProjOptMatches >= nProjOptMatches)
                        {
                            int nNumKFs = 0;
                            // Check the Sim3 transformation with the current KeyFrame covisibles
                            vector<KeyFrame*> vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                            int j = 0;
                            while(nNumKFs < 3 && j<vpCurrentCovKFs.size())
                            {
                                KeyFrame* pKFj = vpCurrentCovKFs[j];
                                cv::Mat mTjc = pKFj->GetPose() * mpCurrentKF->GetPoseInverse();
                                g2o::Sim3 gSjc(Converter::toMatrix3d(mTjc.rowRange(0, 3).colRange(0, 3)),Converter::toVector3d(mTjc.rowRange(0, 3).col(3)),1.0);
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int numProjMatches_j = 0;
                                vector<MapPoint*> vpMatchedMPs_j;
                                bool bValid = DetectCommonRegionsFromLastKF(pKFj,pMostBoWMatchesKF, gSjw,numProjMatches_j, vpMapPoints, vpMatchedMPs_j);

                                if(bValid)
                                {
                                    nNumKFs++;
                                }

                                j++;
                            }

                            if(nNumKFs < 3)
                            {
                                vnStage[index] = 8;
                                vnMatchesStage[index] = nNumKFs;
                            }

                            if(nBestMatchesReproj < numProjOptMatches)
                            {
                                nBestMatchesReproj = numProjOptMatches;
                                nBestNumCoindicendes = nNumKFs;
                                pBestMatchedKF = pMostBoWMatchesKF;
                                g2oBestScw = gScw;
                                vpBestMapPoints = vpMapPoints;
                                vpBestMatchedMapPoints = vpMatchedMP;
                            }


                        }

                    }

                }
            }
        }
        index++;
    }

    if(nBestMatchesReproj > 0)
    {
        pLastCurrentKF = mpCurrentKF;
        nNumCoincidences = nBestNumCoindicendes;
        pMatchedKF2 = pBestMatchedKF;
        pMatchedKF2->SetNotErase();
        g2oScw = g2oBestScw;
        vpMPs = vpBestMapPoints;
        vpMatchedMPs = vpBestMatchedMapPoints;

        return nNumCoincidences >= 3;
    }
    else
    {
        int maxStage = -1;
        int maxMatched;
        for(int i=0; i<vnStage.size(); ++i)
        {
            if(vnStage[i] > maxStage)
            {
                maxStage = vnStage[i];
                maxMatched = vnMatchesStage[i];
            }
        }

    }
    return false;
}

bool LoopClosing::DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
{
    set<MapPoint*> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end()); //Matched map point들을 container 형태로 선언 
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs); //위의 선언된 map point들을 pCurrentKF, pMatchedKF에서 matching하고 최적화

    int nProjMatches = 30; //기준치 선언
    if(nNumProjMatches >= nProjMatches)
    {

        return true;
    }

    return false;
}

int LoopClosing::FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
                                         set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints,
                                         vector<MapPoint*> &vpMatchedMapPoints)
{
    int nNumCovisibles = 5; //covisible 기준치 선언
    vector<KeyFrame*> vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles); //MatchedKF에서 기준치보다 작은 사이즈의 mvpOrderedConnectedKeyFrames를 반환.
                                                                                            //mvpOrderedConnectedKeyFrames는 best covisible KF을 추출하는 함수에서 나온 KF이다.
    int nInitialCov = vpCovKFm.size(); //초기 covisible KF 선언
    vpCovKFm.push_back(pMatchedKFw); //pMatchedKFw push back
    set<KeyFrame*> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end()); //spCheckKFs container 선언
    set<KeyFrame*> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames(); //spCurrentCovisbles container 선언, CurrentKF에서 GetConnectedKeyFrames 즉, weight 값을 가져옴.
    for(int i=0; i<nInitialCov; ++i) //for문 시작
    {
        vector<KeyFrame*> vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles); //위 함수와 동일
        int nInserted = 0; //시작점
        int j = 0; //시작점
        while(j < vpKFs.size() && nInserted < nNumCovisibles) //while문 시작
        {
            if(spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() && spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end())
            {
                spCheckKFs.insert(vpKFs[j]); //위 조건문 통과하면 spCheckKFs에 입력하고 
                ++nInserted; //nInserted값 증가
            }
            ++j; //j 증가
        }
        vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end()); //container에 insert
    }
    set<MapPoint*> spMapPoints; //Mappoint container 선언
    vpMapPoints.clear(); //Map point clear
    vpMatchedMapPoints.clear(); //vpMatchedMapPoints clear
    for(KeyFrame* pKFi : vpCovKFm) //for문 시작
    {
        for(MapPoint* pMPij : pKFi->GetMapPointMatches()) //GetMapPointMatches 추출
        {
            if(!pMPij || pMPij->isBad()) //결국 vpCovKFm에서 뽑은 pMPij가 is Bad이거나 없거나 하면 
                continue; //그냥 지나간다.

            if(spMapPoints.find(pMPij) == spMapPoints.end()) //pMPij와 end값 비교하고 (초기값은 end)
            {
                spMapPoints.insert(pMPij); //같다면 spMapPoints에 pMPij insert
                vpMapPoints.push_back(pMPij); //vpMapPointsdpeh insert
            }
        }
    }

    cv::Mat mScw = Converter::toCvMat(g2oScw);

    ORBmatcher matcher(0.9, true);

    vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
    int num_matches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

    return num_matches; //pCurrentKF와 vpMapPoint, vpMatchedMapPoints matching! and optimize
}

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    // Local Mapping 쪽에 Stop Signal을 보내고
    // Loop correcting을 진행할 때, 새로운 KeyFrame이 들어오는 것을 막아줍니다.
    mpLocalMapper->RequestStop();
    mpLocalMapper->EmptyQueue(); // mlNewKeyFrames에 데이터가 없애기 위해 사용하는 함수


    // If a Global Bundle Adjustment is running, abort it
    // Global Bundle Adjustment가 수행될 경우, 이를 중단합니다.
    cout << "Request GBA abort" << endl;
    if(isRunningGBA())      // GBA가 실행되고 있는지 확인 : mbRunningGBA이라는 변수의 상태 확인
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;   // mbStopGBA를 true로 바꾸므로써, 중단했다는 것을 알려줌

        mnFullBAIdx++;  // Bundle Adjustment Index 1 증가

        if(mpThreadGBA)
        {
            cout << "GBA running... Abort!" << endl;
            mpThreadGBA->detach();  // 쓰레드를 떼어내서 독립적으로 작동하게끔한다.
            delete mpThreadGBA; // 쓰레드 메모리 해제 - 참고 : C++ 스레드의 분류와 사용법[https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=ghoism51&logNo=220190125981]  
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())  // Stop일 때까지 while문을 계속 돈다.
    {
        usleep(1000);   // usleep을 이용하여 Local Mapping이 Stop할 때가지 잠시 대기 1000 micro second
    }

    // Ensure current keyframe is updated
    cout << "start updating connections" << endl;
    mpCurrentKF->UpdateConnections();   
    // 현재 Stop된 상태의 Local Mapping, Global Bundleadjustment가 중단된 상태에서
    // KeyFrame의 covisibility graph를 update

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    // 현재 키프레임에 연결된 키프레임을 검색하고 전파(propagation)를 통해 수정된 Sim3 포즈를 계산합니다.

    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();    
    // Current KeyFrame에서 Covisibility graph로 연결된 Key Frame을 Vector 형태로 담아둡니다.
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);  // Current Key Frame까지 mvpCurrentConnectedKFs 변수에 담아둡니다.

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oLoopScw; // map 자료구조의 초기화 방법 - Current KeyFrame의 Pose를 mg2oLoopScw로 대입
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();    // Current Key Frame의 Camera to World 값 대입

    Map* pLoopMap = mpCurrentKF->GetMap();  // pLoopMap이라는 변수에 Current Key Frame의 Map을 대입
    // 중괄호로 Mutex를 lock하는 범위를 지정해준다.
    // 참고 : https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=muri1004&logNo=221276270566
    {
        // Get Map Mutex
        unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

        const bool bImuInit = pLoopMap->isImuInitialized(); // IMU가 Initialzied 되어 있으면 true, 되어 있지 않으면 false로 선언

        // for문을 활용해서 mvpCurrentConnectedKFs를 순회
        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;  // pointer를 활용하여 vector iterator (= KeyFrame)을 가리킨다.

            cv::Mat Tiw = pKFi->GetPose();  // world to i(현재 index의 KeyFrame)의 Pose를 가져온다.

            if(pKFi!=mpCurrentKF)   // Current Key Frame이 아니라면
            {
                cv::Mat Tic = Tiw*Twc;  // i 번째 Key Frame의 pose와 Current Key Frame의 pose의 차이를 Transformation matrix로 저장
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);  // i 번째 Key Frame의 pose와 Current Key Frame의 Rotation matrix
                cv::Mat tic = Tic.rowRange(0,3).col(3); // i 번째 Key Frame의 pose와 Current Key Frame의 Translation vector

                // g2o 설명 : https://goodgodgd.github.io/ian-flow/archivers/how-to-use-g2o#3-how-to-use-g2o-with-example
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);    
                // i 번째 Key Frame의 pose와 Current Key Frame의 pose의 Sim3 초기화
                
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oLoopScw; 
                // i 번째 Key Frame의 pose와 Current Key Frame의 pose의 Sim3와 Loop의 world to camera Sim3값으로
                // world to i 번째 Key Frame pose의 Sim3값을 계산

                //Pose corrected with the Sim3 of the loop closure
                // Loop closure의 Sim3로 수정된 포즈
                CorrectedSim3[pKFi]=g2oCorrectedSiw;    // CorrectedSim3 자료구조에 대입
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);  // i 번째 Key Frame의 pose와 Current Key Frame의 Rotation matrix
            cv::Mat tiw = Tiw.rowRange(0,3).col(3); //  i 번째 Key Frame의 pose와 Current Key Frame의 Translation vector
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            // world to i 번째 Key Frame의 pose의 Sim3 초기화

            //Pose without correction
            // Loop closure를 활용하지 않은 pose
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        // Loop의 다른 쪽과 정렬되도록 Current Key Frame과 이웃한 Key Frame에서 관찰된 Map point를 수정합니다.

        // for문을 활용해서 CorrectedSim3를 순회(결국 인접한 프레임을 순회)
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;    // i번째 Key Frame을 pointer로 가리킨다.
            g2o::Sim3 g2oCorrectedSiw = mit->second;    // world to i번째 Key Frame의 Sim3값을 pointer로 가리킨다. (Loop을 활용)
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();  // i번째 Key Frame to world의 Sim3값을 pointer로 가리킨다. (Loop을 활용)

            g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];   // world to i번째 Key Frame의 Sim3값을 pointer로 가리킨다. (Loop을 활용 X)

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();  // i번째 Key Frame에서 Map point를 vector에 담는다.
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)    // for문을 활용하여 Map point를 순회
            {
                MapPoint* pMPi = vpMPsi[iMP];   // 하나의 Map point를 pointer로 가리킨다.
                if(!pMPi)  // Map point가 없으면 
                    continue;   // Skip
                if(pMPi->isBad())   // Map point가 좋지 않으면
                    continue;   // Skip
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)    // 현재 Current Key Frame에서의 Map point일 경우
                    continue;   // Skip

                // Project with non-corrected pose and project back with corrected pose
                // 수정되지 않은 포즈로 투영하고 수정된 포즈로 다시 투영합니다.

                cv::Mat P3Dw = pMPi->GetWorldPos(); // Map point의 3D 좌표 값을 가져온다. 
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);    // Eigen matrix 형태로 형변환 후 eigP3D에 저장

                // g2o::Sim3 의 map()함수 설명 : http://docs.ros.org/en/melodic/api/orb_slam2_ros/html/structg2o_1_1Sim3.html
                // map의 input : x,y,z 좌표 >> output : s*(r*xyz) + t;
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));
                // Loop를 활용하여 계산된 Scale 값을 고려해서 x,y,z 값을 계산

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw); // Cv Mat 형태로 형변환 후 cvCorrectedP3D에 저장
                pMPi->SetWorldPos(cvCorrectedP3Dw); // Map point의 3D 좌표 값을 Update
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;  // 수정을 하는데 기준이 된 Key Frame을 Current Key Frame으로 대입
                pMPi->mnCorrectedReference = pKFi->mnId;    // 수정을 하는데 reference가 된 Key Frame을 i번째 Key Frame으로 대입
                pMPi->UpdateNormalAndDepth();   // Normal vector 및 Depth값 update
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            // 수정된 Sim3로 키프레임 포즈를 업데이트합니다. 먼저 Sim3를 SE3로 변환(스케일 변환)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();   // Loop를 활용해 계산된 Sim3에서 Rotation matrix 추출
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();   // Loop를 활용해 계산된 Sim3에서 Translation vector 추출
            double s = g2oCorrectedSiw.scale(); // Loop를 활용해 계산된 Sim3에서 scale 값 추출

            eigt *=(1./s); //[R t/s;0 1] scale 값으로 나눈다.

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);   // world to i번째 Camera pose의 4x4 Transformation Matrix 변환

            pKFi->SetPose(correctedTiw);    // 계산된 Transformation Matrix로 Pose 지정

            // Correct velocity according to orientation correction
            // Orientation 보정에 따른 Velocity 수정
            if(bImuInit)    // IMU가 Initialzied 되어 있으면
            {
                Eigen::Matrix3d Rcor = eigR.transpose()*g2oSiw.rotation().toRotationMatrix();
                // Loop를 활용해 계산된 Sim3에서 Rotation matrix의 역행렬 *  world to i번째 Key Frame의 Sim3(Loop 활용 X) >> 수정된 Rotation Matrix
                pKFi->SetVelocity(Converter::toCvMat(Rcor)*pKFi->GetVelocity());    // 수정된 Rotation Matrix로 Velocity를 새로 update
            }

            // Make sure connections are updated
            pKFi->UpdateConnections(); // KeyFrame의 covisibility graph를 update
        }
        // TODO Check this index increasement
        pLoopMap->IncreaseChangeIndex();    // Map의 index를 update


        // Start Loop Fusion
        // Update matched map points and replace if duplicated  

        // Loop fusion을 시작
        // 서로 matched된 map points를 업데이트하고 복제되는 경우 교체한다.
        for(size_t i=0; i<mvpLoopMatchedMPs.size(); i++)    // for문을 통해 mvpLoopMatchedMPs를 순회
        {
            if(mvpLoopMatchedMPs[i])    // Map point가 존재할 경우
            {
                MapPoint* pLoopMP = mvpLoopMatchedMPs[i];   // Loop의 Map point
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i); // Current Key Frame의 Map point
                if(pCurMP)  // Current Key Frame의 Map point가 존재할 경우
                    pCurMP->Replace(pLoopMP);   // Loop Map point로 교체

                else    // Current Key Frame의 Map point가 없을 경우
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);    // Current Key Frame에 Map point 추가
                    pLoopMP->AddObservation(mpCurrentKF,i); // Key Frame에서 Map point에 대해 Observation 추가
                    pLoopMP->ComputeDistinctiveDescriptors();   // Descriptor 계산
                                                                // Map-point는 여러 KF에서 관찰 될 수 있지만,
                                                                // 이중 가장 대표적인 descirptor를 사용하기 위해서, 갱신 또는 유지하는 작업이 포함됨
                }
            }
        }
    }

    // == 2021-11-18 발표 (LoopClosing::CorrectLoop()) ==
    // 이전 발표 요약: CurrKF, CovisibilityKF, LoopKF의 world 기준의 좌표계 관리 및 MapPoint 좌표계 관리
    // 오늘 발표 요약: 이전 발표에서 관리된 데이터를 이용하여 1) LoopClosing 수행 및 2) GBA 수행

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    
    // LoopKF 주변에서 관찰된 MapPoints를 corrected Pose를 사용하여 현재-키프레임과 인접-키프레임에 투영
    // CorrectedSim3는 [CurrKF, NeighKF]에 해당하는 g2oSiw의 Pose를 포함
    // mvpLoopMapPoints는 DetectCommonRegionsFromBoW()함수에서 결정됨
    // 이 함수에서 loopKF의 LoopMapPoint들과 CurrKF과 neighKF과 연결됨.
    SearchAndFuse(CorrectedSim3, mvpLoopMapPoints); // LoopKF =projection=> CurrKF, neighKF

    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    // MapPoint fusion 이후 루프의 양쪽을 연결하는 covisibility graph의 새 링크를 생성
    map<KeyFrame*, set<KeyFrame*> > LoopConnections; //<KF_, KF_연결된 KFs>

    // CurrKF과 covisibility graph로 연결된 KF를 순회    
    // 여기까지는 CurrKF과 NeighKF 및 LoopKF들이 연결이 되어있음
    // 아래 for문을 통해서 CurrKF의 Nighbor과의 관계를 끊어내고, LoopKF과의 관계만 남김
    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)    
    {
        KeyFrame* pKFi = *vit; // KF정보를 포인터로 할당
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames(); // 할당된 KF에 대한 covisibility graph에 해당하는 KF들을 가져옴

        // Update connections. Detect new links.
        // pKFi에 해당하는 covisibility graph와 essential graph간의 연결관계 업데이트
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames(); // 현재 KF과 연결된 KF들을 map<>형태로 그룹화

        // NeighKF과 연결된 covisibility graph KF를 제거
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {                        
            LoopConnections[pKFi].erase(*vit_prev);
        }

        // CurrKF과 연결된 covisibility graph KF를 제거
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    bool bFixedScale = mbFixScale; // 스케일을 고정할지 안할지??

    // IMU_mono이고 초기화가 안된경우
    if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
        bFixedScale=false;

    // IMU를 사용 && IMU가 초기화 완료된 경우
    if(pLoopMap->IsInertial() && pLoopMap->isImuInitialized())
    {
        // (x,y,z,yaw)값을 최적화
        Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
    }
    else // IMU를 사용하지 않는 경우
    {
        // (x,y,z,roll,pitch,yaw,scale)값을 최적화
        // "Fast Relocalisation and Loop Closing in Keyframe-Based SLAM"
        Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixedScale);
    }

    // 큰 변화에 대한 알림
    // Map::mnBigChangeIdx++;를 함으로써 큰 변화에 대한 인덱스를 증가시킴(default값은 0)
    // bool System::MapChanged() 함수에서 추가적인 처리가 수행됨
    mpAtlas->InformNewBigChange();

    // Add loop edge
    // CurrKF와 LoopKF에 대하여 Loop edge를 생성해줌
    mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
    // GBA를 수행하기 위하여 새로운 쓰레드를 생성 및 수행
    // "True" == (IMU 초기화 || KF 개수 200개 미만 && AtalsMap 개수 1개)
    if(!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1))
    {
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;
        
        // GBA 수행
        // LoopKF과 CurrKF 및 covisibility KF들의 pose와 map-point들을 최적화
        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
    }

    // Loop closed. Release Local Mapping.
    //^
    mpLocalMapper->Release();

    mLastLoopKFid = mpCurrentKF->mnId; //TODO old varible, it is not use in the new algorithm
}

void LoopClosing::MergeLocal()
{
    Verbose::PrintMess("MERGE: Merge Visual detected!!!!", Verbose::VERBOSITY_NORMAL);

    int numTemporalKFs = 15;

    //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
    KeyFrame* pNewChild;
    KeyFrame* pNewParent;

    vector<KeyFrame*> vpLocalCurrentWindowKFs;
    vector<KeyFrame*> vpMergeConnectedKFs;

    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
    bool bRelaunchBA = false;

    Verbose::PrintMess("MERGE: Check Full Bundle Adjustment", Verbose::VERBOSITY_DEBUG);
    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
        bRelaunchBA = true;
    }

    Verbose::PrintMess("MERGE: Request Stop Local Mapping", Verbose::VERBOSITY_DEBUG);
    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    Verbose::PrintMess("MERGE: Local Map stopped", Verbose::VERBOSITY_DEBUG);

    mpLocalMapper->EmptyQueue();

    // Merge map will become in the new active map with the local window of KFs and MPs from the current map.
    // Later, the elements of the current map will be transform to the new active map reference, in order to keep real time tracking
    Map* pCurrentMap = mpCurrentKF->GetMap();
    Map* pMergeMap = mpMergeMatchedKF->GetMap();

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    //Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
    set<KeyFrame*> spLocalWindowKFs;
    //Get MPs in the welding area from the current map
    set<MapPoint*> spLocalWindowMPs;
    if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
    {
        KeyFrame* pKFi = mpCurrentKF;
        int nInserted = 0;
        while(pKFi && nInserted < numTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;

            set<MapPoint*> spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
        }

        pKFi = mpCurrentKF->mNextKF;
        while(pKFi)
        {
            spLocalWindowKFs.insert(pKFi);

            set<MapPoint*> spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
        }
    }
    else
    {
        spLocalWindowKFs.insert(mpCurrentKF);
    }

    vector<KeyFrame*> vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
    spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
    const int nMaxTries = 3;
    int nNumTries = 0;
    while(spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
    {
        vector<KeyFrame*> vpNewCovKFs;
        vpNewCovKFs.empty();
        for(KeyFrame* pKFi : spLocalWindowKFs)
        {
            vector<KeyFrame*> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
            for(KeyFrame* pKFcov : vpKFiCov)
            {
                if(pKFcov && !pKFcov->isBad() && spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end())
                {
                    vpNewCovKFs.push_back(pKFcov);
                }

            }
        }

        spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
        nNumTries++;
    }

    for(KeyFrame* pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        set<MapPoint*> spMPs = pKFi->GetMapPoints();
        spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
    }

    set<KeyFrame*> spMergeConnectedKFs;
    if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
    {
        KeyFrame* pKFi = mpMergeMatchedKF;
        int nInserted = 0;
        while(pKFi && nInserted < numTemporalKFs)
        {
            spMergeConnectedKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;
        }

        pKFi = mpMergeMatchedKF->mNextKF;
        while(pKFi)
        {
            spMergeConnectedKFs.insert(pKFi);
        }
    }
    else
    {
        spMergeConnectedKFs.insert(mpMergeMatchedKF);
    }
    vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
    spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
    nNumTries = 0;
    while(spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
    {
        vector<KeyFrame*> vpNewCovKFs;
        for(KeyFrame* pKFi : spMergeConnectedKFs)
        {
            vector<KeyFrame*> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
            for(KeyFrame* pKFcov : vpKFiCov)
            {
                if(pKFcov && !pKFcov->isBad() && spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end())
                {
                    vpNewCovKFs.push_back(pKFcov);
                }

            }
        }

        spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
        nNumTries++;
    }

    set<MapPoint*> spMapPointMerge;
    for(KeyFrame* pKFi : spMergeConnectedKFs)
    {
        set<MapPoint*> vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
    }

    vector<MapPoint*> vpCheckFuseMapPoint;
    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    cv::Mat Twc = mpCurrentKF->GetPoseInverse();

    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
    cv::Mat twc = Twc.rowRange(0,3).col(3);
    g2o::Sim3 g2oNonCorrectedSwc(Converter::toMatrix3d(Rwc),Converter::toVector3d(twc),1.0);
    g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
    g2o::Sim3 g2oCorrectedScw = mg2oMergeScw;

    KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
    vCorrectedSim3[mpCurrentKF]=g2oCorrectedScw;
    vNonCorrectedSim3[mpCurrentKF]=g2oNonCorrectedScw;

    for(KeyFrame* pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
        {
            continue;
        }

        g2o::Sim3 g2oCorrectedSiw;

        if(pKFi!=mpCurrentKF)
        {
            cv::Mat Tiw = pKFi->GetPose();
            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            vNonCorrectedSim3[pKFi]=g2oSiw;

            cv::Mat Tic = Tiw*Twc;
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            g2oCorrectedSiw = g2oSic*mg2oMergeScw;
            vCorrectedSim3[pKFi]=g2oCorrectedSiw;
        }
        else
        {
            g2oCorrectedSiw = g2oCorrectedScw;
        }
        pKFi->mTcwMerge  = pKFi->GetPose();

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();

        pKFi->mfScale = s;
        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

        pKFi->mTcwMerge = correctedTiw;

        if(pCurrentMap->isImuInitialized())
        {
            Eigen::Matrix3d Rcor = eigR.transpose()*vNonCorrectedSim3[pKFi].rotation().toRotationMatrix();
            pKFi->mVwbMerge = Converter::toCvMat(Rcor)*pKFi->GetVelocity();
        }

    }

    for(MapPoint* pMPi : spLocalWindowMPs)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        KeyFrame* pKFref = pMPi->GetReferenceKeyFrame();
        g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
        g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

        // Project with non-corrected pose and project back with corrected pose
        cv::Mat P3Dw = pMPi->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(eigP3Dw));
        Eigen::Matrix3d eigR = g2oCorrectedSwi.rotation().toRotationMatrix();
        Eigen::Matrix3d Rcor = eigR * g2oNonCorrectedSiw.rotation().toRotationMatrix();

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);

        pMPi->mPosMerge = cvCorrectedP3Dw;
        pMPi->mNormalVectorMerge = Converter::toCvMat(Rcor) * pMPi->GetNormal();
    }

    {
        unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
        unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

        for(KeyFrame* pKFi : spLocalWindowKFs)
        {
            if(!pKFi || pKFi->isBad())
            {
                continue;
            }

            pKFi->mTcwBefMerge = pKFi->GetPose();
            pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
            pKFi->SetPose(pKFi->mTcwMerge);

            // Make sure connections are updated
            pKFi->UpdateMap(pMergeMap);
            pKFi->mnMergeCorrectedForKF = mpCurrentKF->mnId;
            pMergeMap->AddKeyFrame(pKFi);
            pCurrentMap->EraseKeyFrame(pKFi);

            if(pCurrentMap->isImuInitialized())
            {
                pKFi->SetVelocity(pKFi->mVwbMerge);
            }
        }

        for(MapPoint* pMPi : spLocalWindowMPs)
        {
            if(!pMPi || pMPi->isBad())
                continue;

            pMPi->SetWorldPos(pMPi->mPosMerge);
            pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
            pMPi->UpdateMap(pMergeMap);
            pMergeMap->AddMapPoint(pMPi);
            pCurrentMap->EraseMapPoint(pMPi);
        }

        mpAtlas->ChangeMap(pMergeMap);
        mpAtlas->SetMapBad(pCurrentMap);
        pMergeMap->IncreaseChangeIndex();
    }


    //Rebuild the essential graph in the local window
    pCurrentMap->GetOriginKF()->SetFirstConnection(false);
    pNewChild = mpCurrentKF->GetParent(); // Old parent, it will be the new child of this KF
    pNewParent = mpCurrentKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    mpCurrentKF->ChangeParent(mpMergeMatchedKF);
    while(pNewChild )
    {
        pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
        KeyFrame * pOldParent = pNewChild->GetParent();

        pNewChild->ChangeParent(pNewParent);

        pNewParent = pNewChild;
        pNewChild = pOldParent;

    }

    //Update the connections between the local window
    mpMergeMatchedKF->UpdateConnections();

    vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    vpMergeConnectedKFs.push_back(mpMergeMatchedKF);
    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    // Project MapPoints observed in the neighborhood of the merge keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);

    // Update connectivity
    for(KeyFrame* pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for(KeyFrame* pKFi : spMergeConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

    bool bStop = false;
    Verbose::PrintMess("MERGE: Start local BA ", Verbose::VERBOSITY_DEBUG);
    vpLocalCurrentWindowKFs.clear();
    vpMergeConnectedKFs.clear();
    std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(), std::back_inserter(vpLocalCurrentWindowKFs));
    std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(), std::back_inserter(vpMergeConnectedKFs));
    if (mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO)
    {
        Optimizer::MergeInertialBA(mpLocalMapper->GetCurrKF(),mpMergeMatchedKF,&bStop, mpCurrentKF->GetMap(),vCorrectedSim3);
    }
    else
    {
        Optimizer::LocalBundleAdjustment(mpCurrentKF, vpLocalCurrentWindowKFs, vpMergeConnectedKFs,&bStop);
    }

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    Verbose::PrintMess("MERGE: Finish the LBA", Verbose::VERBOSITY_DEBUG);


    ////
    //Update the non critical area from the current map to the merged map
    vector<KeyFrame*> vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
    vector<MapPoint*> vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

    if(vpCurrentMapKFs.size() == 0)
    {
        Verbose::PrintMess("MERGE: There are not KFs outside of the welding area", Verbose::VERBOSITY_DEBUG);
    }
    else
    {
        Verbose::PrintMess("MERGE: Calculate the new position of the elements outside of the window", Verbose::VERBOSITY_DEBUG);
        //Apply the transformation
        {
            if(mpTracker->mSensor == System::MONOCULAR)
            {
                unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information

                for(KeyFrame* pKFi : vpCurrentMapKFs)
                {
                    if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                    {
                        continue;
                    }

                    g2o::Sim3 g2oCorrectedSiw;

                    cv::Mat Tiw = pKFi->GetPose();
                    cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
                    cv::Mat tiw = Tiw.rowRange(0,3).col(3);
                    g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
                    //Pose without correction
                    vNonCorrectedSim3[pKFi]=g2oSiw;

                    cv::Mat Tic = Tiw*Twc;
                    cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                    cv::Mat tic = Tic.rowRange(0,3).col(3);
                    g2o::Sim3 g2oSim(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                    g2oCorrectedSiw = g2oSim*mg2oMergeScw;
                    vCorrectedSim3[pKFi]=g2oCorrectedSiw;

                    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
                    Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
                    double s = g2oCorrectedSiw.scale();

                    pKFi->mfScale = s;
                    eigt *=(1./s); //[R t/s;0 1]

                    cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

                    pKFi->mTcwBefMerge = pKFi->GetPose();
                    pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

                    pKFi->SetPose(correctedTiw);

                    if(pCurrentMap->isImuInitialized())
                    {
                        Eigen::Matrix3d Rcor = eigR.transpose()*vNonCorrectedSim3[pKFi].rotation().toRotationMatrix();
                        pKFi->SetVelocity(Converter::toCvMat(Rcor)*pKFi->GetVelocity()); // TODO: should add here scale s
                    }

                }
                for(MapPoint* pMPi : vpCurrentMapMPs)
                {
                    if(!pMPi || pMPi->isBad()|| pMPi->GetMap() != pCurrentMap)
                        continue;

                    KeyFrame* pKFref = pMPi->GetReferenceKeyFrame();
                    g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
                    g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

                    // Project with non-corrected pose and project back with corrected pose
                    cv::Mat P3Dw = pMPi->GetWorldPos();
                    Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                    Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(eigP3Dw));

                    cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                    pMPi->SetWorldPos(cvCorrectedP3Dw);

                    pMPi->UpdateNormalAndDepth();
                }
            }
        }

        mpLocalMapper->RequestStop();
        // Wait until Local Mapping has effectively stopped
        while(!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }

        // Optimize graph (and update the loop position for each element form the begining to the end)
        if(mpTracker->mSensor != System::MONOCULAR)
        {
            Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs, vpLocalCurrentWindowKFs, vpCurrentMapKFs, vpCurrentMapMPs);
        }


        {
            // Get Merge Map Mutex
            unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

            for(KeyFrame* pKFi : vpCurrentMapKFs)
            {
                if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                {
                    continue;
                }

                // Make sure connections are updated
                pKFi->UpdateMap(pMergeMap);
                pMergeMap->AddKeyFrame(pKFi);
                pCurrentMap->EraseKeyFrame(pKFi);
            }

            for(MapPoint* pMPi : vpCurrentMapMPs)
            {
                if(!pMPi || pMPi->isBad())
                    continue;

                pMPi->UpdateMap(pMergeMap);
                pMergeMap->AddMapPoint(pMPi);
                pCurrentMap->EraseMapPoint(pMPi);
            }
        }
    }

    mpLocalMapper->Release();

    Verbose::PrintMess("MERGE:Completed!!!!!", Verbose::VERBOSITY_DEBUG);

    if(bRelaunchBA && (!pCurrentMap->isImuInitialized() || (pCurrentMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1)))
    {
        // Launch a new thread to perform Global Bundle Adjustment
        Verbose::PrintMess("Relaunch Global BA", Verbose::VERBOSITY_DEBUG);
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;
        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this, pMergeMap, mpCurrentKF->mnId);
    }

    mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
    mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

    pCurrentMap->IncreaseChangeIndex();
    pMergeMap->IncreaseChangeIndex();

    mpAtlas->RemoveBadMaps();

}

void LoopClosing::MergeLocal2()
{
    cout << "Merge detected!!!!" << endl;   // Merge할 부분을 찾았다고 출력

    int numTemporalKFs = 11; //TODO (set by parameter): Temporal KFs in the local window if the map is inertial.
    // MergeLocal()에서는 15로 초기화. IMU를 활용하면 조금 더 적은 KeyFrames로 merge를 진행 

    //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
    // Essential graph를 재구축하기 위한 관계 : 두번 사용 1. Local window에서 사용, 2. 나머지 맵에서 사용
    KeyFrame* pNewChild;        // 새로운 자식 노드 Frame pointer
    KeyFrame* pNewParent;       // 새로운 부모 노드 Frame pointer

    vector<KeyFrame*> vpLocalCurrentWindowKFs;   // 이 변수는 MergeLocal2()에서는 쓰이지 않음. (MergeLocal에서만 사용)
    vector<KeyFrame*> vpMergeConnectedKFs;       // Merge할 때 연결되는 KeyFrame을 모아두기 위한 Vector

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;    // KeyFrameAndPose: map<KeyFrame*, g2o::Sim3>
    // NonCorrectedSim3[mpCurrentKF]=mg2oLoopScw;

    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
    // 실행 중인 BA를 중지한 경우에만 true인 플래그입니다. 이 경우 Merge가 끝날 때 다시 시작해야 합니다.
    bool bRelaunchBA = false;

    cout << "Check Full Bundle Adjustment" << endl; 
    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())      // GBA가 실행되고 있는지 확인 : mbRunningGBA이라는 변수의 상태 확인
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;   // mbStopGBA를 true로 바꾸므로써, 중단했다는 것을 알려줌

        mnFullBAIdx++;  // Bundle Adjustment Index 1 증가

        if(mpThreadGBA)
        {
            cout << "GBA running... Abort!" << endl;
            mpThreadGBA->detach();  // 쓰레드를 떼어내서 독립적으로 작동하게끔한다.
            delete mpThreadGBA; // 쓰레드 메모리 해제 - 참고 : C++ 스레드의 분류와 사용법[https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=ghoism51&logNo=220190125981]  
        }
        bRelaunchBA = true; // 실행중인 Bundle Adjustment를 중지했으므로 true로 값을 바꿈
    }


    cout << "Request Stop Local Mapping" << endl;
    mpLocalMapper->RequestStop();   // Stop일 때까지 while문을 계속 돈다.
    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    cout << "Local Map stopped" << endl;

    // 병합 맵(Merge Map)은 현재 맵(Current Map)의 KF 및 MP의 로컬 창(Local window)과 함께 새 활성 맵(new active map)이 됩니다.
    // 나중에 실시간 추적을 유지하기 위해 현재 지도(Current Map)의 요소가 새로운 활성 지도(new active map) 참조로 변환됩니다.
    Map* pCurrentMap = mpCurrentKF->GetMap();
    Map* pMergeMap = mpMergeMatchedKF->GetMap();

    {
        // 변수명 on은 old_new의 줄임말로 추정
        float s_on = mSold_new.scale(); // Local Window KeyFrame to Current Key Frame의 Sim3에서 scale 값  
        cv::Mat R_on = Converter::toCvMat(mSold_new.rotation().toRotationMatrix()); // Sim3에서 Rotation Matrix
        cv::Mat t_on = Converter::toCvMat(mSold_new.translation()); // Sim3에서 translation vector

        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate); // lock으로 Current Map이 변하지 않도록 한다.

        // LocalMapping에서  mlNewKeyFrames에 데이터가 없애기 위해 사용하는 함수 
        // (Current Key Frame에 mlNewKeyFrames의 첫번째 원소를 계속 대입)
        mpLocalMapper->EmptyQueue();    
        
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        bool bScaleVel=false;   // Scaled Velocity값(scale이 1이 아니면 true로 바뀜)
        if(s_on!=1)
            bScaleVel=true;
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(R_on,s_on,bScaleVel,t_on);
        mpTracker->UpdateFrameIMU(s_on,mpCurrentKF->GetImuBias(),mpTracker->GetLastKeyFrame());
        // Current Frame의 IMU값을 update

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    }

    const int numKFnew=pCurrentMap->KeyFramesInMap();   // Current Map에 있는 Map point들의 갯수를 가져온다.

    // Sensor가 IMU_monocular거나 IMU_stereo일 경우, mbIMU_BA2라는 변수가 false일 경우 (LocalMapping::Run()에서 "start VIBA 2" 다음 true로 변경)
    if((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO)&& !pCurrentMap->GetIniertialBA2()){
        // Map is not completly initialized
        // Map은 완전히 초기화 되지 않았다고 판단
        Eigen::Vector3d bg, ba; // 우선 0으로 초기화
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        Optimizer::InertialOptimization(pCurrentMap,bg,ba); 
        // 두번째 껏 :  void static InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
        // Current Map 최적화 (bias 값 업데이트)
        IMU::Bias b (ba[0],ba[1],ba[2],bg[0],bg[1],bg[2]);  // 바뀐 bias을 활용해서 b라는 변수로 초기화
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate); // Map update가 되지 않도록 lock
        mpTracker->UpdateFrameIMU(1.0f,b,mpTracker->GetLastKeyFrame()); // IMU와 관련된 값들을 Key Frame에 update

        // Set map initialized
        // IMU와 Map에 관련된 변수들 초기화
        pCurrentMap->SetIniertialBA2();
        pCurrentMap->SetIniertialBA1();
        pCurrentMap->SetImuInitialized();

    }

    // Load KFs and MPs from merge map
    // KeyFrames와 Map points들을 Merge map으로 부터 가져옵니다.
    {
        // Get Merge Map Mutex (This section stops tracking!!)
        // Merge Map을 lock을 걸어둔다. (Tracking을 멈춘다.)
        unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
        // Merge 정보를 이용하여 Current Map을 Update한다.
        unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map
        // Merge map update - Old map에서 얻은 Key Frames와 Map points들을 제거한다.

        vector<KeyFrame*> vpMergeMapKFs = pMergeMap->GetAllKeyFrames(); // Merge Map으로부터 Key Frames를 가져와 vector에 저장 (pointer 형식)
        vector<MapPoint*> vpMergeMapMPs = pMergeMap->GetAllMapPoints(); // Merge Map으로부터 Map points들을 가져와 vector에 저장 (pointer 형식)


        for(KeyFrame* pKFi : vpMergeMapKFs) // Merge map Key Frames에 있는 Key Frames들을 순회
        {
            // Key Frame이 좋지 않거나, Merge map에서 가져온 것이 아닐 때
            if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
            {
                continue;   // Skip
            }

            // Make sure connections are updated
            pKFi->UpdateMap(pCurrentMap);   // Key Frame의 Map을 Current Map으로 update
            pCurrentMap->AddKeyFrame(pKFi); // Current Map에도 Key Frame 추가
            pMergeMap->EraseKeyFrame(pKFi); // Merge map에 있는 Key Frame 제거
        }
        // 위 for문의 결과 : Merge Map Key Frames >> Current Map Key Frames로 업데이트

        for(MapPoint* pMPi : vpMergeMapMPs) // Merge map Map points에 있는 Map point들을 순회
        {
            // Map point가 좋지 않거나, Merge map에서 가져온 것이 아닐 때
            if(!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                continue; // Skip

            pMPi->UpdateMap(pCurrentMap);   // Map point의 Map을 Current Map으로 update
            pCurrentMap->AddMapPoint(pMPi); // Current Map에도 Map point 추가
            pMergeMap->EraseMapPoint(pMPi); // Merge map에 있는 Map point 제거
        }
        // 위 for문의 결과 : Merge Map Map points >> Current Map Map points로 업데이트

        // Save non corrected poses (already merged maps)
        // NonCorrectedSim3 저장
        vector<KeyFrame*> vpKFs = pCurrentMap->GetAllKeyFrames();   // Current Map에서 KeyFrame을 모두 Vector에 저장
        for(KeyFrame* pKFi : vpKFs) // Key Frame들을 순회
        {
            cv::Mat Tiw=pKFi->GetPose();    // world to i번째 KeyFrame의 transformation matrix (4x4)
            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);  // world to i번째 KeyFrame의 Rotatoin matrix
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);         // world to i번째 KeyFrame의 Translation vector
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);    // Sim3변수 초기화 (scale 1.0)
            NonCorrectedSim3[pKFi]=g2oSiw;  // NonCorrectedSim3에 대입
        }
    }

    // Essential Graph Rebuilding

    // Q. mpMergeMatchedKF는 Merge Map에 있는 Key Frame 하나를 pointer로 가르키는 건지?
    // 보충 설명 자료 : https://docs.google.com/presentation/d/1vIQHRSs-igaN05jzvCQnff68pF1Uq8kETocFvo0LxbM/edit?usp=sharing

    pMergeMap->GetOriginKF()->SetFirstConnection(false);    // Merge Map에 initial Key Frame을 First Connection으로 지정하지 않습니다.
    pNewChild = mpMergeMatchedKF->GetParent(); // Old parent, it will be the new child of this KF
    pNewParent = mpMergeMatchedKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    mpMergeMatchedKF->ChangeParent(mpCurrentKF);    // Parent Key Frame에 Add Child()
    while(pNewChild)
    {
        pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
        KeyFrame * pOldParent = pNewChild->GetParent();
        pNewChild->ChangeParent(pNewParent);
        pNewParent = pNewChild;
        pNewChild = pOldParent;
    }

    vector<MapPoint*> vpCheckFuseMapPoint; // MapPoint vector from current map to allow to fuse duplicated points with the old map (merge)
    vector<KeyFrame*> vpCurrentConnectedKFs;

    mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);
    vector<KeyFrame*> aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    mvpMergeConnectedKFs.insert(mvpMergeConnectedKFs.end(), aux.begin(), aux.end());
    if (mvpMergeConnectedKFs.size()>6)
        mvpMergeConnectedKFs.erase(mvpMergeConnectedKFs.begin()+6,mvpMergeConnectedKFs.end());

    mpCurrentKF->UpdateConnections();
    vpCurrentConnectedKFs.push_back(mpCurrentKF);
    aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(), aux.end());
    if (vpCurrentConnectedKFs.size()>6)
        vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin()+6,vpCurrentConnectedKFs.end());

    set<MapPoint*> spMapPointMerge;
    for(KeyFrame* pKFi : mvpMergeConnectedKFs)
    {
        set<MapPoint*> vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
        if(spMapPointMerge.size()>1000)
            break;
    }

    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

    for(KeyFrame* pKFi : vpCurrentConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for(KeyFrame* pKFi : mvpMergeConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

    if (numKFnew<10){
        mpLocalMapper->Release();
        return;
    }

    // Perform BA
    bool bStopFlag=false;
    KeyFrame* pCurrKF = mpTracker->GetLastKeyFrame();
    Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap,CorrectedSim3);

    // Release Local Mapping.
    mpLocalMapper->Release();


    return;
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint*> &vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        int num_replaces = 0;
        KeyFrame* pKFi = mit->first;
        Map* pMap = pKFi->GetMap();

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(vpMapPoints.size(),static_cast<MapPoint*>(NULL));
        int numFused = matcher.Fuse(pKFi,cvScw,vpMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {


                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);

            }
        }

        total_replaces += num_replaces;
    }
}


void LoopClosing::SearchAndFuse(const vector<KeyFrame*> &vConectedKFs, vector<MapPoint*> &vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    for(auto mit=vConectedKFs.begin(), mend=vConectedKFs.end(); mit!=mend;mit++)
    {
        int num_replaces = 0;
        KeyFrame* pKF = (*mit);
        Map* pMap = pKF->GetMap();
        cv::Mat cvScw = pKF->GetPose();

        vector<MapPoint*> vpReplacePoints(vpMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,vpMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);
            }
        }
        total_replaces += num_replaces;
    }
}



void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::RequestResetActiveMap(Map *pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMapRequested = true;
        mpMapToReset = pMap;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetActiveMapRequested)
                break;
        }
        usleep(3000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        cout << "Loop closer reset requested..." << endl;
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
        mbResetActiveMapRequested = false;
    }
    else if(mbResetActiveMapRequested)
    {

        for (list<KeyFrame*>::const_iterator it=mlpLoopKeyFrameQueue.begin(); it != mlpLoopKeyFrameQueue.end();)
        {
            KeyFrame* pKFi = *it;
            if(pKFi->GetMap() == mpMapToReset)
            {
                it = mlpLoopKeyFrameQueue.erase(it);
            }
            else
                ++it;
        }

        mLastLoopKFid=mpAtlas->GetLastInitKFid();
        mbResetActiveMapRequested=false;

    }
}

void LoopClosing::RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF)
{
    Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);

    const bool bImuInit = pActiveMap->isImuInitialized();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartFGBA = std::chrono::steady_clock::now();
#endif

    if(!bImuInit)
        Optimizer::GlobalBundleAdjustemnt(pActiveMap,10,&mbStopGBA,nLoopKF,false);
    else
        Optimizer::FullInertialBA(pActiveMap,7,false,nLoopKF,&mbStopGBA);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartMapUpdate = std::chrono::steady_clock::now();

    double timeFullGBA = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_StartMapUpdate - time_StartFGBA).count();
    vTimeFullGBA_ms.push_back(timeFullGBA);
#endif


    int idx =  mnFullBAIdx;

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!bImuInit && pActiveMap->isImuInitialized())
            return;

        if(!mbStopGBA)
        {
            Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
            Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

            mpLocalMapper->RequestStop();
            
            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(),pActiveMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(!pChild || pChild->isBad())
                        continue;

                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;

                        cv::Mat Rcor = pChild->mTcwGBA.rowRange(0,3).colRange(0,3).t()*pChild->GetRotation();
                        if(!pChild->GetVelocity().empty()){
                            pChild->mVwbGBA = Rcor*pChild->GetVelocity();
                        }
                        else
                            Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);


                        pChild->mBiasGBA = pChild->GetImuBias();


                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);

                if(pKF->bImu)
                {
                    pKF->mVwbBefGBA = pKF->GetVelocity();
                    if (pKF->mVwbGBA.empty())
                        Verbose::PrintMess("pKF->mVwbGBA is empty", Verbose::VERBOSITY_NORMAL);

                    assert(!pKF->mVwbGBA.empty());
                    pKF->SetVelocity(pKF->mVwbGBA);
                    pKF->SetNewBias(pKF->mBiasGBA);                    
                }

                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = pActiveMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    if(pRefKF->mTcwBefGBA.empty())
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }

            pActiveMap->InformNewBigChange();
            pActiveMap->IncreaseChangeIndex();

            mpLocalMapper->Release();

            Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndMapUpdate = std::chrono::steady_clock::now();

    double timeMapUpdate = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMapUpdate - time_StartMapUpdate).count();
    vTimeMapUpdate_ms.push_back(timeMapUpdate);

    double timeGBA = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMapUpdate - time_StartFGBA).count();
    vTimeGBATotal_ms.push_back(timeGBA);
#endif
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
