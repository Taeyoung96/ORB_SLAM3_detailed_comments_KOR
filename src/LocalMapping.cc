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


#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"
#include "Config.h"

#include<mutex>
#include<chrono>

namespace ORB_SLAM3
{

LocalMapping::LocalMapping(System* pSys, Atlas *pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName):
    mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial), mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas), bInitializing(false),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
    mbNewInit(false), mIdxInit(0), mScale(1.0), mInitSect(0), mbNotBA1(true), mbNotBA2(true), infoInertial(Eigen::MatrixXd::Zero(9,9))
{
    mnMatchesInliers = 0;

    mbBadImu = false;

    mTinit = 0.f;

    mNumLM = 0;
    mNumKFCulling=0;

#ifdef REGISTER_TIMES
    nLBA_exec = 0;
    nLBA_abort = 0;
#endif

}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames() && !mbBadImu)
        {

#ifdef REGISTER_TIMES
            double timeLBA_ms = 0;
            double timeKFCulling_ms = 0;

            std::chrono::steady_clock::time_point time_StartProcessKF = std::chrono::steady_clock::now();
#endif
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndProcessKF = std::chrono::steady_clock::now();

            double timeProcessKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndProcessKF - time_StartProcessKF).count();
            vdKFInsert_ms.push_back(timeProcessKF);
#endif

            // Check recent MapPoints
            MapPointCulling();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCulling = std::chrono::steady_clock::now();

            double timeMPCulling = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCulling - time_EndProcessKF).count();
            vdMPCulling_ms.push_back(timeMPCulling);
#endif
            // Triangulate new MapPoints
            CreateNewMapPoints();

            mbAbortBA = false;

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCreation = std::chrono::steady_clock::now();

            double timeMPCreation = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCreation - time_EndMPCulling).count();
            vdMPCreation_ms.push_back(timeMPCreation);
#endif

            bool b_doneLBA = false;
            int num_FixedKF_BA = 0;
            int num_OptKF_BA = 0;
            int num_MPs_BA = 0;
            int num_edges_BA = 0;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                if(mpAtlas->KeyFramesInMap()>2)
                {

                    if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
                    {
                        float dist = cv::norm(mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()) +
                                cv::norm(mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter());

                        if(dist>0.05)
                            mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                        {
                            if((mTinit<10.f) && (dist<0.02))
                            {
                                cout << "Not enough motion for initializing. Reseting..." << endl;
                                unique_lock<mutex> lock(mMutexReset);
                                mbResetRequestedActiveMap = true;
                                mpMapToReset = mpCurrentKeyFrame->GetMap();
                                mbBadImu = true;
                            }
                        }

                        bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);
                        Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
                        b_doneLBA = true;
                    }
                    else
                    {
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA);
                        b_doneLBA = true;
                    }

                }
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndLBA = std::chrono::steady_clock::now();

                if(b_doneLBA)
                {
                    timeLBA_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLBA - time_EndMPCreation).count();
                    vdLBASync_ms.push_back(timeLBA_ms);

                    nLBA_exec += 1;
                    if(mbAbortBA)
                    {
                        nLBA_abort += 1;
                    }
                    vnLBA_edges.push_back(num_edges_BA);
                    vnLBA_KFopt.push_back(num_OptKF_BA);
                    vnLBA_KFfixed.push_back(num_FixedKF_BA);
                    vnLBA_MPs.push_back(num_MPs_BA);
                }

#endif

                // Initialize IMU here
                if(!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
                {
                    if (mbMonocular)
                        InitializeIMU(1e2, 1e10, true);
                    else
                        InitializeIMU(1e2, 1e5, true);
                }


                // Check redundant local Keyframes
                KeyFrameCulling();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndKFCulling = std::chrono::steady_clock::now();

                timeKFCulling_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndKFCulling - time_EndLBA).count();
                vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif

                if ((mTinit<100.0f) && mbInertial)
                {
                    if(mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState==Tracking::OK)
                    {
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()){
                            if (mTinit>5.0f)
                            {
                                cout << "start VIBA 1" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
                                if (mbMonocular)
                                    InitializeIMU(1.f, 1e5, true);
                                else
                                    InitializeIMU(1.f, 1e5, true);

                                cout << "end VIBA 1" << endl;
                            }
                        }
                        else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()){
                            if (mTinit>15.0f){
                                cout << "start VIBA 2" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
                                if (mbMonocular)
                                    InitializeIMU(0.f, 0.f, true);
                                else
                                    InitializeIMU(0.f, 0.f, true);

                                cout << "end VIBA 2" << endl;
                            }
                        }

                        // scale refinement
                        if (((mpAtlas->KeyFramesInMap())<=100) &&
                                ((mTinit>25.0f && mTinit<25.5f)||
                                (mTinit>35.0f && mTinit<35.5f)||
                                (mTinit>45.0f && mTinit<45.5f)||
                                (mTinit>55.0f && mTinit<55.5f)||
                                (mTinit>65.0f && mTinit<65.5f)||
                                (mTinit>75.0f && mTinit<75.5f))){
                            cout << "start scale ref" << endl;
                            if (mbMonocular)
                                ScaleRefinement();
                            cout << "end scale ref" << endl;
                        }
                    }
                }
            }

#ifdef REGISTER_TIMES
            vdLBA_ms.push_back(timeLBA_ms);
            vdKFCulling_ms.push_back(timeKFCulling_ms);
#endif


            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);


#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLocalMap = std::chrono::steady_clock::now();

            double timeLocalMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLocalMap - time_StartProcessKF).count();
            vdLMTotal_ms.push_back(timeLocalMap);
#endif
        }
        else if(Stop() && !mbBadImu)
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::EmptyQueue()
{
    while(CheckNewKeyFrames())
        ProcessNewKeyFrame();
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    int borrar = mlpRecentAddedMapPoints.size();

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;

        if(pMP->isBad())
            lit = mlpRecentAddedMapPoints.erase(lit);
        else if(pMP->GetFoundRatio()<0.25f)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
        {
            lit++;
            borrar--;
        }
    }
    //cout << "erase MP: " << borrar << endl;
}

/* !
* @brief Current KeyFrame을 기준으로 인접한 Keyframe을 이용하여 triangulation을 수행하여 3D point를 생성하고
*        Atlas-map과 current-map 객체에 3D point를 등록
*        사용되는 위치: LocalMapping::Run();
*/
void LocalMapping::CreateNewMapPoints()
{
    // Stereo인 경우
    int nn = 10; // nn은 새로운 MapPoint를 생성하기 위하여 가져올 인접 keyframe의 개수
    
    // Mono인경우
    if(mbMonocular)
        nn=20;   // nn은 새로운 MapPoint를 생성하기 위하여 가져올 인접 keyframe의 개수

    // Retrieve neighbor keyframes in covisibility graph
    // covisibility 그래프에서 인접 키프레임 검색
    //   - Stereo인 경우 10개의 인접 keyframe을 가져옴
    //   - mono인   경우 20개의 인접 keyframe을 가져옴
    //   - vector의 index가 낮을 수록 현재 keyframe과 가까운 위치에 존재함
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    // IMU인 경우
    if (mbInertial)
    {
        KeyFrame* pKF = mpCurrentKeyFrame; // Current Keyframe을 획득
        
        // while문에서 for문 같이 종료조건을 위한 count        
        int count=0;

        // 반복 조건은 2개
        //   1) 0부터 인접 keyframe의 개수만큼 수행됨
        //   2) 인접 키프레임의 previousKeyFrame이 존재할 때 수행됨
        while((vpNeighKFs.size()<=nn)&&(pKF->mPrevKF)&&(count++<nn))
        {
            // pKF의 이전 Previous KeyFrame을 탐색
            vector<KeyFrame*>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
            if(it==vpNeighKFs.end()) // 마지막 인접 keyframe에 도달했을 경우, previous-Keyframe을 추가함
                vpNeighKFs.push_back(pKF->mPrevKF);
            pKF = pKF->mPrevKF; // prevoius-keyframe의 포인터를 넘김
        }
    }

    // ORBmatcher의 threshold를 설정 (0.6)
    float th = 0.6f;

    ORBmatcher matcher(th,false); // ORBmatcher 생성

    auto Rcw1 = mpCurrentKeyFrame->GetRotation_();          // CurrentKeyframe의 Rotatiom matrix를 가져옴(3x3)
    auto Rwc1 = Rcw1.t();                                   // transpose를 수행
    auto tcw1 = mpCurrentKeyFrame->GetTranslation_();       // CurrentKeyframe의 translate matrix를 가져옴(3x1)
    cv::Matx44f Tcw1{Rcw1(0,0),Rcw1(0,1),Rcw1(0,2),tcw1(0), // Transforamtion matirx을 생성 (4x4)
                     Rcw1(1,0),Rcw1(1,1),Rcw1(1,2),tcw1(1),
                     Rcw1(2,0),Rcw1(2,1),Rcw1(2,2),tcw1(2),
                     0.f,0.f,0.f,1.f};

    auto Ow1 = mpCurrentKeyFrame->GetCameraCenter_();       // Current Keyframe의 카메라 중심값을 가져옴

    // Getting intrinsic parameters of camera
    const float &fx1 = mpCurrentKeyFrame->fx;        // fx
    const float &fy1 = mpCurrentKeyFrame->fy;        // fy
    const float &cx1 = mpCurrentKeyFrame->cx;        // cx
    const float &cy1 = mpCurrentKeyFrame->cy;        // cy
    const float &invfx1 = mpCurrentKeyFrame->invfx;  // invfx = 1.0f/fx; (Frame.cc-167 lines or  검색)       
    const float &invfy1 = mpCurrentKeyFrame->invfy;  // invfy = 1.0f/fy; (Frame.cc-168 lines or  검색)       

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor; // Scale Factor에 1.5를 곱해줌

    // Search matches with epipolar restriction and triangulate
    // 에피폴라 제한이 있는 검색 일치 및 삼각 측량
    for(size_t i=0; i<vpNeighKFs.size(); i++) // 인접 keyframe만큼 반복문 수행
    {
        if(i>0 && CheckNewKeyFrames()) // i가 0보다 작거나, 새로운 keyframe이 없으면 종료
            return;

        KeyFrame* pKF2 = vpNeighKFs[i]; // 인접 keyframe을 가져옴

        // pCamera1: CurrentKeyframe
        // pCamera2: Neighbor Keyframe
        GeometricCamera* pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;

        // Check first that baseline is not too short
        // baseline의 길이가 짧은지 확인
        auto Ow2 = pKF2->GetCameraCenter_();        // 인접한 Keyframe에 대한 CameraCenter  가져옴 (3x1 matrix)
        auto vBaseline = Ow2-Ow1;                   // LastKeyframe(Ow1)과 인접한 Keyframe(Ow2)의 카메라 중심점 차이를 계산
        const float baseline = cv::norm(vBaseline); // norm-함수를 이용하여 거리 계산

        // Stereo, Stereo-IMU, RGBD인 경우, 즉 Mono 또는 Mono-IMU가 아닌경우
        // 인접 keyframe의 baseline이 초기 Baseline인 보다 작으면, Skip
        if(!mbMonocular)
        {
            // 인접 keyframe의 baseline이 초기 Baseline인 보다 작으면, Skip
            if(baseline<pKF2->mb) 
            continue;
        }
        // Mono 또는 Mono-IMU인 경우
        else
        {
            // pKF2에서 획득된 depth들의 중앙값을 가져옴.
            // ComputeSceneMedianDepth function:
            //    1. pKF2에서 획득된 point들의 depth값들을 오름차순으로 sorting을 진행
            //    2. sorting된 depth에서 중앙값을 가져옴
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);

            // Baseline을 depth 중앙값으로 나누어 BaselineDepth의 ratio을 계산
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            // ratioBaselineDepth 값이 0.01보다 작으면 skip
            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        // 인접 keyframe과 lastkeyframe간의 fundamental matrix를 계산
        // 결과적으로 다음식을 만족하는 F12를 구하게 됨
        // mpCurrentKeyFrame.point.t() * F12 * pKF2.point = 0
        auto F12 = ComputeF12_(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        // 에피폴라 제약조건을 이용한 매칭 탐색
        vector<pair<size_t,size_t> > vMatchedIndices;

        // 1.1 + IMU
        // 1.2 + GetIniertialBA2(): LocalMapping.cc 202번 line 참조
        //                       ->LocalMapping 수행중 current-keyframe과 previous-keyframe의 시간 차이가 15 미만이면 true
        // 1.3 + GetIniertialBA1(): LocalMapping 수행중 current-keyframe과 previous-keyframe의 시간 차이가  5 미만이면 true
        //  or
        // 2. + Tracking을 실패 했을 경우
        bool bCoarse = mbInertial                                           
                      && ((!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()  
                      &&    mpCurrentKeyFrame->GetMap()->GetIniertialBA1())
                      ||    mpTracker->mState==Tracking::RECENTLY_LOST);

        // SearchForTriangulation(): ORBmatcher.cc의 1208번 line의 함수
        // Fundamental matrix를 이용하여 tracking에 실패한 point들에 대하여 매칭정보를 추가적으로 탐색
        matcher.SearchForTriangulation_(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false,bCoarse);

        auto Rcw2 = pKF2->GetRotation_();                        // PrevKeyframe의 Rotatiom matrix를 가져옴(3x3)
        auto Rwc2 = Rcw2.t();                                    // transpose를 수행
        auto tcw2 = pKF2->GetTranslation_();                     // PrevKeyframe의 translate matrix를 가져옴(3x1)
        cv::Matx44f Tcw2{Rcw2(0,0),Rcw2(0,1),Rcw2(0,2),tcw2(0),  // Transforamtion matirx을 생성 (4x4) 
                         Rcw2(1,0),Rcw2(1,1),Rcw2(1,2),tcw2(1),  
                         Rcw2(2,0),Rcw2(2,1),Rcw2(2,2),tcw2(2),  
                         0.f,0.f,0.f,1.f};

        // camera parameter 가져옴
        const float &fx2 = pKF2->fx;        // fx
        const float &fy2 = pKF2->fy;        // fy
        const float &cx2 = pKF2->cx;        // cx
        const float &cy2 = pKF2->cy;        // cy
        const float &invfx2 = pKF2->invfx;  // invfx = 1.0f/fx; (Frame.cc-167 lines or  검색)       
        const float &invfy2 = pKF2->invfy;  // invfy = 1.0f/fy; (Frame.cc-168 lines or  검색)       

        // Triangulate each match
        const int nmatches = vMatchedIndices.size(); // fundamental matrix를 이용하여 tracking에 실패한 point들을 재추적 정보
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;  // Prev KF의 point id
            const int &idx2 = vMatchedIndices[ikp].second; // Curr KF의 point id

            // Curr KF의 matching point가 -1 인경우
            //   true)  Curr KF의 idx1의 KeysUn 값을 가져옴
            //   false) Curr KF의 matching point가 idx1보다 작은경우
            //              true) Curr KF의 idx1의 KeysUn 값을 가져옴
            //             false) Curr KF의 right->(idx1-currKF.NLeft) 의 KeysUn 값을 가져옴 // 어떤 의미인지 몰르겠음
            // mvKeysRight는 fisheye에서만 사용되는 변수 (//KeyPoints in the right image (for stereo fisheye, coordinates are needed))
            const cv::KeyPoint &kp1 = (mpCurrentKeyFrame -> NLeft == -1) ? mpCurrentKeyFrame->mvKeysUn[idx1]
                                                                         : (idx1 < mpCurrentKeyFrame -> NLeft) ? mpCurrentKeyFrame -> mvKeys[idx1]
            
                                                                                                               : mpCurrentKeyFrame -> mvKeysRight[idx1 - mpCurrentKeyFrame -> NLeft];
            // mvuRight: negative value for monocular points
            // current KF의 negative value 값을 가져옴
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1]; // Right가 모든 센서에서 사용되는 것 같음 (확인필요)
            
            // fisheye가 아니고, negative value가 존재하는 경우
            bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur>=0); 
            const bool bRight1 = (mpCurrentKeyFrame -> NLeft == -1 || idx1 < mpCurrentKeyFrame -> NLeft) ? false
                                                                               : true;
            // Neighbor KF의 matching point가 -1 인경우
            //   true)  Neighbor KF의 idx1의 KeysUn 값을 가져옴
            //   false) Neighbor KF의 matching point가 idx2보다 작은경우
            //              true) Neighbor KF의 idx2의 KeysUn 값을 가져옴
            //             false) Neighbor KF의 right->(idx2-NeighborKF.NLeft) 의 KeysUn 값을 가져옴 // 어떤 의미인지 몰르겠음
            // (return) = (조건문) ? (true) : (false)
            const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                            : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
                                                                                     : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];

            // mvuRight: negative value for monocular points
            // Neighbor KF의 negative value 값을 가져옴
            const float kp2_ur = pKF2->mvuRight[idx2];
            
            // fisheye가 아니고, negative value가 존재하는 경우
            bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur>=0);
            const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false
                                                                               : true;

            // Fisheye 경우
            if(mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2){          
                if(bRight1 && bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRightRotation_();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetRightTranslation_();
                    Tcw1 = mpCurrentKeyFrame->GetRightPose_();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter_();

                    Rcw2 = pKF2->GetRightRotation_();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetRightTranslation_();
                    Tcw2 = pKF2->GetRightPose_();
                    Ow2 = pKF2->GetRightCameraCenter_();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera2;
                }
                else if(bRight1 && !bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRightRotation_();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetRightTranslation_();
                    Tcw1 = mpCurrentKeyFrame->GetRightPose_();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter_();

                    Rcw2 = pKF2->GetRotation_();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetTranslation_();
                    Tcw2 = pKF2->GetPose_();
                    Ow2 = pKF2->GetCameraCenter_();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera;
                }
                else if(!bRight1 && bRight2){
                    Rcw1 = mpCurrentKeyFrame->GetRotation_();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetTranslation_();
                    Tcw1 = mpCurrentKeyFrame->GetPose_();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter_();

                    Rcw2 = pKF2->GetRightRotation_();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetRightTranslation_();
                    Tcw2 = pKF2->GetRightPose_();
                    Ow2 = pKF2->GetRightCameraCenter_();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera2;
                }
                else{
                    Rcw1 = mpCurrentKeyFrame->GetRotation_();
                    Rwc1 = Rcw1.t();
                    tcw1 = mpCurrentKeyFrame->GetTranslation_();
                    Tcw1 = mpCurrentKeyFrame->GetPose_();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter_();

                    Rcw2 = pKF2->GetRotation_();
                    Rwc2 = Rcw2.t();
                    tcw2 = pKF2->GetTranslation_();
                    Tcw2 = pKF2->GetPose_();
                    Ow2 = pKF2->GetCameraCenter_();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera;
                }
            }

            // Check parallax between rays
            // kp1.pt (2x1)를 3차원 값으로 unproject (Curr KF)
            // kp2.pt (2x1)를 3차원 값으로 unproject (Neighbor KF)
            // unprojectMat_()의 출력 값은 (3x1)로 3번째 row값(z)는 1으로 고정
            // unprojectMat_()는 3d point에 대한 ray 정보를 계산
            auto xn1 = pCamera1->unprojectMat_(kp1.pt); // (X, Y, 1)
            auto xn2 = pCamera2->unprojectMat_(kp2.pt);

            auto ray1 = Rwc1*xn1; // world 좌표계로 변환
            auto ray2 = Rwc2*xn2; // world 좌표계로 변환

            // 두 ray의 cos(theta) 값을 계산
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;  // cos(theata) 결과를 양수화
            float cosParallaxStereo1 = cosParallaxStereo; // 초기값 대입
            float cosParallaxStereo2 = cosParallaxStereo; // 초기값 대입

            // bStereo1: Curr KF가 fisheye가 아니고, negative value가 존재하는 경우
            // 카메라 원점과 3D point간의 cos 값 계산
            // mb값은 고정
            // depth값이 클수록 atan2의 값은 더 커짐
            // 그러므로 depth값이 클수록 cos값은 더 작아짐
            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));

            // bStereo2: Neigbor KF가 fisheyeCreateNewMapPoints가 아니고, negative value가 존재하는 경우
            // 카메라 원점과 3D point간의 cos 값 계산
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            // 두 값중 작은 값을 반환
            // 두 값중 더 작은 값이, 카메라로부터 더 멀리 떨어져 있는 것을 의미함.
            // stereo가 아니면, cosParallaxStereo1, cosParallaxStereo2가 동일
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);       

            cv::Matx31f x3D; // Triangulation 결과 값에 대한 변수 (3x1, XYZ)
            bool bEstimated = false; // Triangulation이 성공했는지 여부
                                     // 성공했다면 (true)가 되고, reprojection error를 비교하고 -> 맵포인트를 추가하는 순서도로 작동
            if(cosParallaxRays<cosParallaxStereo 
               && cosParallaxRays>0                                                // rotation이 심하지 않은 경우
               && (bStereo1 || bStereo2 || (cosParallaxRays<0.9998 && mbInertial)  // bStereo1, bStereo2는 stereo를 의미(fisheye 아님)
               || (cosParallaxRays<0.9998 && !mbInertial)))                        // cosParallaxRay < 0.9998 : ray들이 평행하지 않음을 의미(평행하면 삼각측량이 부정확) 
            {
                // DLT를 이용한 선형 삼각측량 방법 
                // Cyrill Stachniss교수님 강의: https://www.youtube.com/watch?v=3NcQbZu6xt8                
                // https://imkaywu.github.io/blog/2017/07/triangulation/                
                // https://www.youtube.com/watch?v=oFZQykvEw14
                cv::Matx14f A_r0 = xn1(0) * Tcw1.row(2) - Tcw1.row(0);
                cv::Matx14f A_r1 = xn1(1) * Tcw1.row(2) - Tcw1.row(1);
                cv::Matx14f A_r2 = xn2(0) * Tcw2.row(2) - Tcw2.row(0);
                cv::Matx14f A_r3 = xn2(1) * Tcw2.row(2) - Tcw2.row(1);
                cv::Matx44f A{A_r0(0), A_r0(1), A_r0(2), A_r0(3),
                              A_r1(0), A_r1(1), A_r1(2), A_r1(3),
                              A_r2(0), A_r2(1), A_r2(2), A_r2(3),
                              A_r3(0), A_r3(1), A_r3(2), A_r3(3)};

                cv::Matx44f u,vt;
                cv::Matx41f w;                
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV); // SVD 계산 (cv::SVD::FULL_UV는 u와 vt가 직교행렬 생성에 대한 flag)

                cv::Matx41f x3D_h = vt.row(3).t(); // 3차원 x

                if(x3D_h(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D_h.get_minor<3,1>(0,0) / x3D_h(3); // [X Y Z 1]
                // 위에랑 아래랑 같은 의미 코드 (opencv 버전에 따라서 위에 것이 컴파일이 안될 수도 있는데, 아래로 수정하면됨)
                //x3D = cv::Matx31f(x3D_h.get_minor<3,1>(0,0)(0) / x3D_h(3), x3D_h.get_minor<3,1>(0,0)(1) /  x3D_h(3), x3D_h.get_minor<3,1>(0,0)(2) / x3D_h(3));
                bEstimated = true;

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo_(idx1);
                bEstimated = true;
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo_(idx2);
                bEstimated = true;
            }
            else
            {
                continue; //No stereo and very low parallax
            }

            cv::Matx13f x3Dt = x3D.t(); // 3D point에 대한 transpose

            if(!bEstimated) continue;
            //Check triangulation in front of cameras
            // 측량값이 카메라 앞쪽에 생성됬는지 아닌지 확인 (Curr KF)
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1(2); // camera coordinate기준으로 z값을 계산 (양수값이 되어야함)
            if(z1<=0) // 음수면 skip
                continue;

            // 측량값이 카메라 앞쪽에 생성됬는지 아닌지 확인 (Neighbor KF)
            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2(2); // camera coordinate기준으로 z값을 계산 (양수값이 되어야함)
            if(z2<=0) // 음수면 skip
                continue;

            // Check reprojection error in first keyframe            
            // Keyframe 에서 특징점들을 추출할 때, 생성되는 sigma or octvae값을 가지고있음
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1(0); // Curr KF 좌표계변환
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1(1); // Curr KF 좌표계변환
            const float invz1 = 1.0/z1; // inverse

            if(!bStereo1) // stereo가 아닌 경우
            {
                cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1,y1,z1)); // Curr KF - image에 투영
                float errX1 = uv1.x - kp1.pt.x; // 오차 계산
                float errY1 = uv1.y - kp1.pt.y; // 오차 계산

                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1) // 에러가 너무 크면 skip
                    continue;

            }
            else // stereo 인 경우
            {
                // 똑같이 오차 계산
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            // curr KF의 오차 계산과 동일함
            // KF2는 neighbor KF를 뜻함
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2,y2,z2));
                float errX2 = uv2.x - kp2.pt.x;
                float errY2 = uv2.y - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            auto normal1 = x3D-Ow1;           // camera center of the curr KF과 3D point의 벡터 계산
            float dist1 = cv::norm(normal1);  // camera center of the curr KF과 3D point의 거리 계산

            auto normal2 = x3D-Ow2;           // camera center of the neighbor KF과 3D point의 벡터 계산
            float dist2 = cv::norm(normal2);  // camera center of the neighbor KF과 3D point의 벡터 계산

            if(dist1==0 || dist2==0)          // 둘중 하나라도 값이 0이면 skip
                continue;

            // mbFarPoints는 bool변수
            // mbFarPoints는 .yaml파일에서 "thFarPoints" 파라미터가 존재하는 경우 true로 설정 / 아니면 false
            // thFarPoints는 일부 .yaml파일에만 존재(example. TUM_512_ourdoors.yaml)
            if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints)) // 거리 값이 mThFarPoints보다 크면 skip
                continue;

            const float ratioDist = dist2/dist1; // 거리 비율 계산
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave]; // ??

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor) // (거리 비율 * 1.5)이 ratioOctave 보다 작으면 skip
                continue;                                                              // (거리 비율)이 (ratioOctave * 1.5) 보다 작으면 skip

            // Triangulation is succesfull
            // 지금까지 skip되지 않고 조건을 만족하면, 삼각측량 값을 3D mappint로 사용
            cv::Mat x3D_(x3D);
            MapPoint* pMP = new MapPoint(x3D_,mpCurrentKeyFrame,mpAtlas->GetCurrentMap());

            pMP->AddObservation(mpCurrentKeyFrame,idx1); // observation 값 추가 // 기존에 존재하는 map-point인지, 새로운 map-point인지 점검과정이 포함됨
            pMP->AddObservation(pKF2,idx2);              // observation 값 추가 // 기존에 존재하는 map-point인지, 새로운 map-point인지 점검과정이 포함됨

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);    // Map point 추가
            pKF2->AddMapPoint(pMP,idx2);                 // Map point 추가

            pMP->ComputeDistinctiveDescriptors();        // Descriptor 계산
                                                         // Map-point는 여러 KF에서 관찰 될 수 있지만,
                                                         // 이중 가장 대표적인 descirptor를 사용하기 위해서, 갱신 또는 유지하는 작업이 포함됨

            pMP->UpdateNormalAndDepth();                 // 3D점에서 카메라까지의 normal vector와 depth값을 update

            mpAtlas->AddMapPoint(pMP);                   // Map point 추가 (AtlasMap)
            mlpRecentAddedMapPoints.push_back(pMP);      // Map point 추가 (RecentMap)
        }
    }
}


void LocalMapping::SearchInNeighbors() //neighbors keyframe을 검색하는 함수입니다. 
{
    // Retrieve neighbor keyframes
    int nn = 10; //nn 10으로 초기화합니다. 
    if(mbMonocular) //monocular일때 nn을 20으로 초기화합니다. 
        nn=20;
    
    /* 
    / current keyframe 기준 저장되어있던 orderedconnectedkeyframes list에서 nn만틈 가져옵니다.
    / GetBestCovisibilityKeyFrames는 mvpOrderedConnectedKeyFrames를 return하는 함수입니다. 
    / nn size보다 mvpOrderedConnectedKeyFrames의 size가 더 크다면 mvpOrderedConnectedKeyFrames의 begin 시점부터 nn개의 keyframe들을 가져옵니다. 
    */
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn); 

    vector<KeyFrame*> vpTargetKFs; //KevFrame을 원소로 하는 vector를 선언합니다. 

    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++) //불러왔던 vpNeighKFs의 첫번째 keyframe부터해서 end까지 for문을 돌립니다. 
    {
        KeyFrame* pKFi = *vit; //KeyFrame pKFi를 선언합니다. 
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId) //불러온 keyframe이 bad flag이거나 mnFuseTargetForKF의 초기값 0과 currentkeyframe id가 동일하면 해당 keyframe은 pass합니다. 
            continue;
        vpTargetKFs.push_back(pKFi); //if문에 해당되지 않는다면 vector인 vpTargetKFs에 push back해줍니다. 
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId; //그리고 mnFuseTargetForKF을 current id로 교체합니다. 
    }

    //따라서 for문을 거치고 나면 vpTargetKFs에 유요한 covisibilitykeyFrame들이 저장되게 됩니다. 

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    for(int i=0, imax=vpTargetKFs.size(); i<imax; i++) //저장한 vector에서 다시 for문을 돌립니다. 
    {
        const vector<KeyFrame*> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20); //SecondNeighKFs vector를 저장한 vector들에게서 순차적으로 가져옵니다. 
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++) //앞에서 진행한 유요한 keyframe가져오는 작업을 한번더 거치게됩니다. 
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2); //저장되어있던 keyframe들과 관련있는 keyframe들이 뒷단에 다시 저장되게 됩니다. 
            pKFi2->mnFuseTargetForKF=mpCurrentKeyFrame->mnId; 
        }
        if (mbAbortBA) //중간에 BA가 들어오거나 keyFrame이 insert되는 경우에는 해당 Neighbors keyframe을 찾는것을 중단합니다. 
            break;
    }
    //따라서 for문을 거치고나면 처음에 current frame의 covisibility와 관련된 keyframe으로 유요한 keyframe을 찾았었는데 그때 찾았던 keyframe을 가지고 covisibility와 관련된 keyframe을 다시 찾아서 vector에 저장되게됩니다. 

    // Extend to temporal neighbors
    if(mbInertial) //imu데이터가 존재할때 실행합니다. (해당 value는 mSensor 변수에 의해 결정됩니다.) 
    {
        KeyFrame* pKFi = mpCurrentKeyFrame->mPrevKF; //previous keyframe을 가져옵니다. 
        while(vpTargetKFs.size()<20 && pKFi) //while문 시작합니다. 
        {
            if(pKFi->isBad() || pKFi->mnFuseTargetForKF==mpCurrentKeyFrame->mnId) //previous keyframe이 bad flag를 가지고있거나 currentkeyframe id와 동일하면 if문이 실행됩니다.
            {
                pKFi = pKFi->mPrevKF; //previous Keyframe으로 교체하고 저장합니다.
                continue;
            }
            vpTargetKFs.push_back(pKFi); //해당 KeyFrame을 vector에 저장합니다.
            pKFi->mnFuseTargetForKF=mpCurrentKeyFrame->mnId; //그리고 마찬가지로 FuseTargetForKF id를 갱신합니다. 
            pKFi = pKFi->mPrevKF; //마찬가지로 if문 구문을 통과하지 않은 keyframe들을 previous keyframe으로 저장합니다.
        }
    }

    // Search matches by projection from current KF in target KFs

    ORBmatcher matcher; //ORB matcher를 선언합니다. 
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches(); //GetMapPointMatches를 vector로 저장하여 사용합니다. 
    //해당 vector를 for문 돌립니다. 
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit; //각 N번째 target KF들을 사용합니다. 

        matcher.Fuse(pKFi,vpMapPointMatches); //target KF과 MapPointMatches의 KF과 비교하여 중복되는 map point를 찾습니다. 
        if(pKFi->NLeft != -1) matcher.Fuse(pKFi,vpMapPointMatches,true); //fisheye camera 일 경우입니다. 
    }

    if (mbAbortBA) //AbortBA flag가 true일때 종료합니다. 즉 새로운 keyframe이 들어오고있으면 return합니다. 
        return;

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates; //Fuse할 후보군을 저장할 vector를 선언합니다. 
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size()); //해당 vector사이즈를 mappoint matches와 targetKF의 size를 곱한 크기로 늘려줍니다.

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++) //위에서 저장했던 TargetKF들을 for문 돌려줍니다. 
    {
        KeyFrame* pKFi = *vitKF; //각각 N번째 Keyframe입니다. 

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches(); //각 Keyframe에서 keypoint와 관련있는 map point를 가져옵니다. 

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++) //해당 map point들을 for문 돌립니다.
        {
            MapPoint* pMP = *vitMP; // N번째 mappoint 선언
            if(!pMP) //map point가 0값이면 continue합니다. 
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId) //mappoint가 bad flag거나 current frame과 중복되면 continue합니다. 
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId; //현재 mappoint의 fuse 후보군을 current Frame id와 똑같이 만듭니다. 업데이트용
            vpFuseCandidates.push_back(pMP); //fuse 후보군 vector에 업데이트합니다. 
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates); //최종적으로 current keyframe과 업데이트를 마친 fuse condidates들과 matcher.fuse합니다. 
    if(mpCurrentKeyFrame->NLeft != -1) matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates,true); //이건 fisheye일때 실행됩니다. 


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches(); //따라서 최종적으로 map point가 업데이트 된 current keyframe의 데이터를 mappointmatches에 불러옵니다.
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++) //해당 mappoint를 for문 돌립니다. 
    {
        MapPoint* pMP=vpMapPointMatches[i]; //N번째 mappoint입니다.
        if(pMP)
        {
            if(!pMP->isBad()) //map point가 정상이면 
            {
                pMP->ComputeDistinctiveDescriptors(); //새로운 mappoint와 기존 mappoint와의 거리를 계산합니다. 
                pMP->UpdateNormalAndDepth(); //normal vector 계산과 max distance, min distance 즉 depth데이터를 계산하여 최신화합니다. 
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections(); //최종 업데이트를 진행합니다. 
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2) //tracking.cc의 compute12와 동일합니다. 
{
    cv::Mat R1w = pKF1->GetRotation(); //1번 KF의 rotation data
    cv::Mat t1w = pKF1->GetTranslation(); //1번 KF의 translation data
    cv::Mat R2w = pKF2->GetRotation(); //2번 KF의 rotation data
    cv::Mat t2w = pKF2->GetTranslation(); //2번 KF의 translation data

    cv::Mat R12 = R1w*R2w.t(); // delta rotation
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w; //delta translation

    cv::Mat t12x = SkewSymmetricMatrix(t12); //skew symmetric matrix 형태로 변환

    const cv::Mat &K1 = pKF1->mpCamera->toK(); //1번 KF의 distortion data
    const cv::Mat &K2 = pKF2->mpCamera->toK(); //2번 KF의 distortion data


    return K1.t().inv()*t12x*R12*K2.inv(); //최종 Fundamental matrix 연산
}

cv::Matx33f LocalMapping::ComputeF12_(KeyFrame *&pKF1, KeyFrame *&pKF2) //Mat과 차이점은 연산량이 적은 작은 matrix일때 사용합니다. 기능은 위의 computeF12와 동일합니다. 
{
    auto R1w = pKF1->GetRotation_();
    auto t1w = pKF1->GetTranslation_();
    auto R2w = pKF2->GetRotation_();
    auto t2w = pKF2->GetTranslation_();

    auto R12 = R1w*R2w.t();
    auto t12 = -R1w*R2w.t()*t2w+t1w;

    auto t12x = SkewSymmetricMatrix_(t12);

    const auto &K1 = pKF1->mpCamera->toK_();
    const auto &K2 = pKF2->mpCamera->toK_();


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop() //Stop이 필요할때 사용하는 함수입니다. 
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true; //local mapping을 중단하는 flag
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true; //Keyframe연산이 될때 stop되는 flag
}

bool LocalMapping::Stop() //간단하게 local mapping이 stop되는 시점에 실행되는 함수입니다. 
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop) //requested가 true거나 Notstop이 false일때 실행됩니다. 
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped() //local mapping이 stop 되어있는지 확인할때 사용하는 함수입니다. 
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested() //stop이 requested가 되었는지 확인할때 사용하는 함수입니다. 
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release() //
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)  // local mapping run이 끝났는지 확인합니다. 
        return;
    mbStopped = false; //stop 관련 bool 타입 변수들을 전부 false처리합니다. 
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++) //여태 들어온 keyframe을 삭제합니다. 
        delete *lit;
    mlNewKeyFrames.clear(); //여태 들어온 keyframe에 속해있는 관련 data, information을 삭제합니다. 

    cout << "Local Mapping RELEASE" << endl; // local mapping한것이 release됩니다. 한국말로는 배포정도...?
}

//tracking.cc 에서 accept상태인지 아닌지 확인할때 사용하는 함수입니다. 
//실질적인 기능은 SetAcceptkeyframs에서 진행하고 마지막으로 true가 되면 해당 함수를 통해 tracking에서 받습니다. 
bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames; //해당 변수도 SetAcceptkeyframes에서 결정합니다. 
}

//local mapping 종료시점을 알려줍니다. 해당 값이 false되어있다면 tracking에서 데이터를 넘기지 않습니다. 
void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag) //tracking에서 
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped) //tracking에서 설정한 flag값과 mbstopped값이 다르면 무조건 false됩니다.  
        return false;

    mbNotStop = flag; //mbNotstop 값을 flag로 저장합니다. 

    return true; //flag와 mbstopped가 같으면 true를 반환합니다. --> 결론적으로 tracking에서 createnewkeyframe진행시 local mapping이 멈춰져있는지 확인하는 작업입니다. 
}

void LocalMapping::InterruptBA() //AbortBA가 true가 되어야할때, 즉 keyframe을 받으면 안될때 실행되는 함수입니다. 
{
    mbAbortBA = true; 
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    const int Nd = 21; // This should be the same than that one from LIBA
    mpCurrentKeyFrame->UpdateBestCovisibles();
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    float redundant_th;
    if(!mbInertial)
        redundant_th = 0.9;
    else if (mbMonocular)
        redundant_th = 0.9;
    else
        redundant_th = 0.5;

    const bool bInitImu = mpAtlas->isImuInitialized();
    int count=0;

    // Compoute last KF from optimizable window:
    unsigned int last_ID;
    if (mbInertial)
    {
        int count = 0;
        KeyFrame* aux_KF = mpCurrentKeyFrame;
        while(count<Nd && aux_KF->mPrevKF)
        {
            aux_KF = aux_KF->mPrevKF;
            count++;
        }
        last_ID = aux_KF->mnId;
    }



    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        count++;
        KeyFrame* pKF = *vit;

        if((pKF->mnId==pKF->GetMap()->GetInitKFid()) || pKF->isBad())
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = (pKF -> NLeft == -1) ? pKF->mvKeysUn[i].octave
                                                                     : (i < pKF -> NLeft) ? pKF -> mvKeys[i].octave
                                                                                          : pKF -> mvKeysRight[i].octave;
                        const map<KeyFrame*, tuple<int,int>> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            tuple<int,int> indexes = mit->second;
                            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
                            int scaleLeveli = -1;
                            if(pKFi -> NLeft == -1)
                                scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
                            else {
                                if (leftIndex != -1) {
                                    scaleLeveli = pKFi->mvKeys[leftIndex].octave;
                                }
                                if (rightIndex != -1) {
                                    int rightLevel = pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
                                    scaleLeveli = (scaleLeveli == -1 || scaleLeveli > rightLevel) ? rightLevel
                                                                                                  : scaleLeveli;
                                }
                            }

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>thObs)
                                    break;
                            }
                        }
                        if(nObs>thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if(nRedundantObservations>redundant_th*nMPs)
        {
            if (mbInertial)
            {
                if (mpAtlas->KeyFramesInMap()<=Nd)
                    continue;

                if(pKF->mnId>(mpCurrentKeyFrame->mnId-2))
                    continue;

                if(pKF->mPrevKF && pKF->mNextKF)
                {
                    const float t = pKF->mNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;

                    if((bInitImu && (pKF->mnId<last_ID) && t<3.) || (t<0.5))
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                    else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && (cv::norm(pKF->GetImuPosition()-pKF->mPrevKF->GetImuPosition())<0.02) && (t<3))
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                }
            }
            else
            {
                pKF->SetBadFlag();
            }
        }
        if((count > 20 && mbAbortBA) || count>100)
        {
            break;
        }
    }
}


cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

cv::Matx33f LocalMapping::SkewSymmetricMatrix_(const cv::Matx31f &v)
{
    cv::Matx33f skew{0.f, -v(2), v(1),
                     v(2), 0.f, -v(0),
                     -v(1), v(0), 0.f};

    return skew;
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Map reset recieved" << endl;
        mbResetRequested = true;
    }
    cout << "LM: Map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Map reset, Done!!!" << endl;
}

void LocalMapping::RequestResetActiveMap(Map* pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Active map reset recieved" << endl;
        mbResetRequestedActiveMap = true;
        mpMapToReset = pMap;
    }
    cout << "LM: Active map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequestedActiveMap)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Active map reset, Done!!!" << endl;
}

void LocalMapping::ResetIfRequested()
{
    bool executed_reset = false;
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            executed_reset = true;

            cout << "LM: Reseting Atlas in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mbResetRequested=false;
            mbResetRequestedActiveMap = false;

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mIdxInit=0;

            cout << "LM: End reseting Local Mapping..." << endl;
        }

        if(mbResetRequestedActiveMap) {
            executed_reset = true;
            cout << "LM: Reseting current map in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mbResetRequestedActiveMap = false;
            cout << "LM: End reseting Local Mapping..." << endl;
        }
    }
    if(executed_reset)
        cout << "LM: Reset free the mutex" << endl;

}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)
{
    if (mbResetRequested)
        return;

    float minTime;
    int nMinKF;
    if (mbMonocular)
    {
        minTime = 2.0;
        nMinKF = 10;
    }
    else
    {
        minTime = 1.0;
        nMinKF = 10;
    }


    if(mpAtlas->KeyFramesInMap()<nMinKF)
        return;

    // Retrieve all keyframe in temporal order
    list<KeyFrame*> lpKF;
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());

    if(vpKF.size()<nMinKF)
        return;

    mFirstTs=vpKF.front()->mTimeStamp;
    if(mpCurrentKeyFrame->mTimeStamp-mFirstTs<minTime)
        return;

    bInitializing = true;

    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();
    IMU::Bias b(0,0,0,0,0,0);

    // Compute and KF velocities mRwg estimation
    if (!mpCurrentKeyFrame->GetMap()->isImuInitialized())
    {
        cv::Mat cvRwg;
        cv::Mat dirG = cv::Mat::zeros(3,1,CV_32F);
        for(vector<KeyFrame*>::iterator itKF = vpKF.begin(); itKF!=vpKF.end(); itKF++)
        {
            if (!(*itKF)->mpImuPreintegrated)
                continue;
            if (!(*itKF)->mPrevKF)
                continue;

            dirG -= (*itKF)->mPrevKF->GetImuRotation()*(*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();
            cv::Mat _vel = ((*itKF)->GetImuPosition() - (*itKF)->mPrevKF->GetImuPosition())/(*itKF)->mpImuPreintegrated->dT;
            (*itKF)->SetVelocity(_vel);
            (*itKF)->mPrevKF->SetVelocity(_vel);
        }

        dirG = dirG/cv::norm(dirG);
        cv::Mat gI = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, -1.0f);
        cv::Mat v = gI.cross(dirG);
        const float nv = cv::norm(v);
        const float cosg = gI.dot(dirG);
        const float ang = acos(cosg);
        cv::Mat vzg = v*ang/nv;
        cvRwg = IMU::ExpSO3(vzg);
        mRwg = Converter::toMatrix3d(cvRwg);
        mTinit = mpCurrentKeyFrame->mTimeStamp-mFirstTs;
    }
    else
    {
        mRwg = Eigen::Matrix3d::Identity();
        mbg = Converter::toVector3d(mpCurrentKeyFrame->GetGyroBias());
        mba = Converter::toVector3d(mpCurrentKeyFrame->GetAccBias());
    }

    mScale=1.0;

    mInitTime = mpTracker->mLastFrame.mTimeStamp-vpKF.front()->mTimeStamp;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg, mba, mbMonocular, infoInertial, false, false, priorG, priorA);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (mScale<1e-1)
    {
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }



    // Before this line we are not changing the map

    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    if ((fabs(mScale-1.f)>0.00001)||!mbMonocular)
    {
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Converter::toCvMat(mRwg).t(),mScale,true);
        mpTracker->UpdateFrameIMU(mScale,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    // Check if initialization OK
    if (!mpAtlas->isImuInitialized())
        for(int i=0;i<N;i++)
        {
            KeyFrame* pKF2 = vpKF[i];
            pKF2->bImu = true;
        }

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    if (bFIBA)
    {
        if (priorA!=0.f)
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, 0, NULL, true, priorG, priorA);
        else
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, 0, NULL, false);
    }

    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();

    // If initialization is OK
    mpTracker->UpdateFrameIMU(1.0,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
    if (!mpAtlas->isImuInitialized())
    {
        cout << "IMU in Map " << mpAtlas->GetCurrentMap()->GetId() << " is initialized" << endl;
        mpAtlas->SetImuInitialized();
        mpTracker->t0IMU = mpTracker->mCurrentFrame.mTimeStamp;
        mpCurrentKeyFrame->bImu = true;
    }


    mbNewInit=true;
    mnKFs=vpKF.size();
    mIdxInit++;

    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    mpTracker->mState=Tracking::OK;
    bInitializing = false;

    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}

void LocalMapping::ScaleRefinement()
{
    // Minimum number of keyframes to compute a solution
    // Minimum time (seconds) between first and last keyframe to compute a solution. Make the difference between monocular and stereo
    // unique_lock<mutex> lock0(mMutexImuInit);
    if (mbResetRequested)
        return;

    // Retrieve all keyframes in temporal order
    list<KeyFrame*> lpKF;
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());

    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();

    mRwg = Eigen::Matrix3d::Identity();
    mScale=1.0;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (mScale<1e-1)
    {
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }

    // Before this line we are not changing the map
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    if ((fabs(mScale-1.f)>0.00001)||!mbMonocular)
    {
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Converter::toCvMat(mRwg).t(),mScale,true);
        mpTracker->UpdateFrameIMU(mScale,mpCurrentKeyFrame->GetImuBias(),mpCurrentKeyFrame);
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    double t_inertial_only = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();

    // To perform pose-inertial opt w.r.t. last keyframe
    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}



bool LocalMapping::IsInitializing()
{
    return bInitializing;
}


double LocalMapping::GetCurrKFTime()
{

    if (mpCurrentKeyFrame)
    {
        return mpCurrentKeyFrame->mTimeStamp;
    }
    else
        return 0.0;
}

KeyFrame* LocalMapping::GetCurrKF()
{
    return mpCurrentKeyFrame;
}

} //namespace ORB_SLAM
