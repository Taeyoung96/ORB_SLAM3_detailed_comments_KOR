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
    //^ Run
    //^ mbFinished : While 문을 돌고 있는지 아닌지를 체크하는 Flag 
    mbFinished = false; //mbFinished는 초기값 true입니다. run 함수가 종료될때 true로 다시 반환합니다.

    while(1)    //while문 시작 
    {
        //^ Cannot accept keyframe now because LM is busy
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);  
        //해당 함수를 false로 넣고 실행하면 AcceptKeyFrames flag가 fasle로 설정됩니다.
        //그렇게되면 tracking에서 CreateNewKeyFrame()를 실행하지 않습니다.

        //^ Check if key frames list is empty
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames() && !mbBadImu)    //checknewkeyframes 함수 : newkeyframe 리스트가 empty인지 아닌지 판단해주는 함수입니다. 
                                                //mbBadimu 함수 imu가 재대로 안들어올때 true를 반환해주게 되어있습니다. 
                                                //해당 두 함수에 관한 true가 형성될때 if문이 가동됩니다.
        {

#ifdef REGISTER_TIMES
            double timeLBA_ms = 0;
            double timeKFCulling_ms = 0;

            std::chrono::steady_clock::time_point time_StartProcessKF = std::chrono::steady_clock::now();
#endif
            //^ Keyframe 전처리
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();   //new keyframe 기본작업을 합니다. 여기서 current keyframe이 update됩니다.
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndProcessKF = std::chrono::steady_clock::now();

            double timeProcessKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndProcessKF - time_StartProcessKF).count();
            vdKFInsert_ms.push_back(timeProcessKF);
#endif
            //^ mlpRecentAddedMapPoints 정리
            //^ Redundant Map Points
            // Check recent MapPoints
            MapPointCulling();      //각 keyframe별 mappoint를 모으는 작업을 합니다. 
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCulling = std::chrono::steady_clock::now();

            double timeMPCulling = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCulling - time_EndProcessKF).count();
            vdMPCulling_ms.push_back(timeMPCulling);
#endif
            //^ MapPoints 생성(in Atlas)
            // Triangulate new MapPoints
            CreateNewMapPoints();   //생성되었던 mappoint들과 새로들어오는 keyframe과 covisibility - keyframe을 가지고 새로운 map point를 생성합니다

            mbAbortBA = false;  //해당 flag는 local mapping을 중단해야할때 true로 전환되는 flag입니다. 
                                //따라서 local mapping을 진행하고 있으므로 false로 선언합니다.

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                //해당함수는 위의 설명대로 neighbor keyframes와 매칭하여 duplications point들을 제거합니다.
                SearchInNeighbors();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCreation = std::chrono::steady_clock::now();

            double timeMPCreation = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCreation - time_EndMPCulling).count();
            vdMPCreation_ms.push_back(timeMPCreation);
#endif

            /*
            아래 변수들을 초기화합니다.
            진행하면서 변수 각각의 설명을 하겠습니다.
            */
            bool b_doneLBA = false;
            int num_FixedKF_BA = 0;
            int num_OptKF_BA = 0;
            int num_MPs_BA = 0;
            int num_edges_BA = 0;

            //^ BA
            if(!CheckNewKeyFrames() && !stopRequested())    //stopRequested flag가 정상이고 CheckNewKeyFrames가 정상적으로 clear 되어있으면
                                                             //if문이 시작됩니다.
            {
                //^ Local BA
                if(mpAtlas->KeyFramesInMap()>2)     //Current map 상에서 사용되고있는 keyframe의 갯수가 3개 이상일때 시작합니다. 
                {

                    if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized()) //imu센서가 포함이 되어있고 imu initialize가 완료되었을때 if문이 실행됩니다.
                                                                                      //isImuInitialized는 local mapping에 imu initialize 함수가 동작한뒤에 실행됩니다.(true)
                                                                                      //mbInertial 변수는 imu가 들어간 모델이면 true가 되게 됩니다. (mono-imu, stereo-imu)
                    {
                        //a,b,c의 시간순서대로의 keyframe이 있을때 (a가 current입니다.)
                        //world좌표계에서의 camera center값을 a keyframe과 b keyframe에서 가져와서 distance를 계산합니다. 
                        //그리고 b keyframe과 c keyframe의 distance도 계산하여 더합니다.
                        float dist = cv::norm(mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()) +
                                cv::norm(mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter()); 
                        
                        //만약 위에서 계산한 dist값이 0.05(5cm)가 넘어가면 mTinit값에 현재 keyframe과 prev keyframe의 timestamp차이값을 추가합니다. 
                        if(dist>0.05)
                            mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;

                        //mapping 시작전에는 당연히 map이 clear가 되어야하고 map clear부분에서 해당 GetIniertialBA2는 false가 됩니다.
                        //따라서 map이 정상적으로 초기화되어있는 경우에는 해당 if문에 들어가게됩니다. 
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                        {   
                            //mTinit 값이 10이하이고 dist값이 0.02보다 적으면 에러메세지를 발생시키고 
                            //badimu flag를 true로 바꾸고 현재 map과 active map을 초기화시킵니다.
                            if((mTinit<10.f) && (dist<0.02))
                            {
                                cout << "Not enough motion for initializing. Reseting..." << endl;
                                unique_lock<mutex> lock(mMutexReset);
                                mbResetRequestedActiveMap = true;
                                mpMapToReset = mpCurrentKeyFrame->GetMap();
                                mbBadImu = true;
                            }
                        }
                        
                        //bLarge변수를 선언합니다. monocular에서 Inlier된 map point가 75개 이상인 경우 or monocular가 아니고 inlier된 map point가 100개 이상인 경우 true값을 반환합니다. 
                        bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);

                        //optimizer 함수를 좀 자세히 들여다 보아야 정확한 실행루트를 이해할 수 있습니다. 
                        //간단하게 설명하자면 LocalInertialBA는 imu센서의 acc, vel, pose, gyro 데이터와 visual의 keypoint, mappoint들을 조합하여 BA를 진행합니다.
                        Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
                        b_doneLBA = true;
                    }
                    else
                    {   //LocalBundleAdjustment는 위의 LocalInertialBA와 다르게 visual data만 이용하여 BA를 진행합니다.
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA);
                        b_doneLBA = true;
                    }

                }//따라서 여기까지 BA 최적화까지 진행함을 알수 있습니다.
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
                //^ IMU Initialization
                // Initialize IMU here
                // imu가 들어가있는 경우인데도 Imu initialized가 되지않았을때 해당 if문이 진행됩니다.
                if(!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
                {
                    if (mbMonocular)    //mono-imu 일때
                        InitializeIMU(1e2, 1e10, true);
                    else                // s나머지
                        InitializeIMU(1e2, 1e5, true);
                }


                // Check redundant local Keyframes
                // 불필요한 keyFrame제거를 위해 KeyFrameCulling 함수를 진행합니다.
                KeyFrameCulling();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndKFCulling = std::chrono::steady_clock::now();

                timeKFCulling_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndKFCulling - time_EndLBA).count();
                vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif
                // mTinit이 100보다 작고 imu 데이터를 가지고있을때 if문이 실행됩니다.
                if ((mTinit<100.0f) && mbInertial)
                {
                    //imu initialized 여부와 tracking 여부를 체크합니다. 성공되었으면 실행합니다.
                    if(mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState==Tracking::OK)
                    {
                        //GetIniertialBA1이 재대로 진행되었는지 여부를 확인합니다.
                        //
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()){
                            //재대로 BA1이 진행되지 않았을때 실행합니다.
                            //mTinit이 5초 이상일때
                            if (mTinit>5.0f)
                            {
                                cout << "start VIBA 1" << endl;
                                //BA1을 실행합니다.
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
                                if (mbMonocular) //만약 mono일때면 
                                    InitializeIMU(1.f, 1e5, true);
                                else //mono가 아니라면 
                                    InitializeIMU(1.f, 1e5, true);
                                //여기서 parameter가 동일한게 들어갑니다. 이유는 잘 모르겠습니다. 어짜피 InitializeIMU안에서 
                                //mono가 따로 돌게됩니다. 

                                cout << "end VIBA 1" << endl;
                                //종료
                            }
                        }
                        //BA2가 재대로 진행되어있는지 봅니다. 재대로 실행되지 않았으면 실행됩니다.
                        else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()){
                            //15초 이후일때 실행됩니다.
                            if (mTinit>15.0f){
                                //BA2 최적화를 진행합니다. 
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
                            //mono일때 scalerefinement를 실행합니다.
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

            //loopcloser에서 새로들어온 currentKeyFrame을 insert 시켜줍니다. (map update)
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);


#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLocalMap = std::chrono::steady_clock::now();

            double timeLocalMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLocalMap - time_StartProcessKF).count();
            vdLMTotal_ms.push_back(timeLocalMap);
#endif
        }
        //^ mlNewKeyFrames가 없을 때
        //^ Stop request가 왔는지 체크
        //local mapping stop sign이 들어왔을때 실행합니다. 
        else if(Stop() && !mbBadImu)
        {
            // Safe area to stop
            //^ Stop 요청이 오면 stop이 풀릴 떄까지(mLocalMapper->Release()가 호출될 때까지)
            //^ 대기
            //^ 예시 : LoopClosing -> CorrectLoop()
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            
            //^ finish request가 왔으면 전체 while 문 out
            if(CheckFinish())
                break;
        }

        //^ Reset 요청 있었다면 LM에서 사용하는 parameter들 Reset 실행
        //^ RequestedReset : tracking에서
        //^ RequestedActiveMapReset : tracking에서
        ResetIfRequested();

        //^ Local mapping done
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        //^ Finish 요청 있으면 LM Loop 종료
        //^ Finish 요청 : system에서 shutdown 일때
        if(CheckFinish())
            break;

        //^ LM 주기
        usleep(3000);
    }

    //^ Stop : while 문 내에서 잠시 대기
    //^ Finish : while 문을 벗어남( = shutdown)

    //^ LM 종료
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
    //CheckNewKeyFrame( !mlNewKeyFrames.empty() 반환 ) while문 수행
    // ProcessNewKeyFrame 수행
    while(CheckNewKeyFrames())
        ProcessNewKeyFrame();
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    // lit = mlpRecentAddedMapPoints의 시작 index
    // CurrentKFid = mpCurrentKeyFrame ID
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    // nThObs = mbMonocular인 경우 2, 그 외에는 3
    // cnThObs = nThObs를 const 형태로 저장
    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    // borrar = mlpRecentAddedMapPoints의 사이즈를 저장
    int borrar = mlpRecentAddedMapPoints.size();

    // lit이 mlpRecentAddedMapPoints의 마지막 index가 될떄까지  while문 수행


    // *pMP = lit의 포인터 저장
    // pMP의 조건에 따라서 pMP와 lit 재정의
    // 1. pMP가 isBad인 경우
    // -- 현재의 mlpRecentAddedMapPoints를 지우고 그 다음 index 반환

    // 2. pMP의 GetFoundRatio가 0.25f
    // -- GetFoundRatio = mnFound/mnVisible
    // -- pMP를 SetBadFlag로 설정
    // -- 현재의 mlpRecentAddedMapPoints를 지우고 그 다음 index 반환

    // 3. CurrentKFid와 pMP의 mnFisrtKFid의 차이가 2 이상인 경우 && pMP의 Observations이 cnThObs보다 작은경우
    // -- pMP를 SetBadFlag로 설정
    // -- 현재의 mlpRecentAddedMapPoints를 지우고 그 다음 index 반환

    // 4. CurrentKFid와 pMP의 mnFisrtKFid의 차이가 3 이상인 경우
    // -- 현재의 mlpRecentAddedMapPoints를 지우고 그 다음 index 반환

    // 5. 그 이외 경우
    // lit 증가
    // borrar 감소

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

void LocalMapping::SearchInNeighbors()
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

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
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

cv::Matx33f LocalMapping::ComputeF12_(KeyFrame *&pKF1, KeyFrame *&pKF2)
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

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;                 //local mapping을 중단하는 flag
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;                       //new keyframe이 새로 insert될때 local mapping 기능을 중단하기 위한 flag
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)       //requested가 true거나 Notstop이 false일때 실행됩니다. 
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
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

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;       //해당 변수도 SetAcceptkeyframes에서 결정합니다. 
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped) //tracking에서 설정한 flag값과 mbstopped값이 다르면 무조건 false됩니다.  
        return false;

    mbNotStop = flag; //mbNotstop 값을 flag로 저장합니다. 

    return true; //flag와 mbstopped가 같으면 true를 반환합니다.
    //  --> 결론적으로 tracking에서 createnewkeyframe진행시 local mapping이 멈춰져있는지 확인하는 작업입니다.
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points

    // Local Keyframes에서 중복되는 Keyframes가 있는지 체크합니다.  
    // 표시되는 MapPoint의 90%가 다른 3개 이상의 키프레임
    // (동일하거나 더 미세한 크기)에서 표시되는 경우 키프레임은 중복된 것으로 간주됩니다.
    // 가까운 스테레오 points에 대해서만 고려합니다.

    const int Nd = 21; // This should be the same than that one from LIBA
    // LIBA는 Local Inertial Bundle Adjustment의 줄임말.
    // LIBA와 같은 KeyFrame의 갯수로 초기화

    mpCurrentKeyFrame->UpdateBestCovisibles();  // KeyFrame간의 관계를 Update (KeyFrames와 Weights를 Update)

    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();  // Local Key Frames 선언

    float redundant_th; // 중복되는 비율 선언

    if(!mbInertial) // IMU를 사용하지 않을 때
        redundant_th = 0.9; // 중복되는 비율 0.9로 선언
    else if (mbMonocular)   // Monocular모드 일 때
        redundant_th = 0.9; // 중복되는 비율 0.9로 선언
    else    // 스테레오 모드 일 때
        redundant_th = 0.5; // 중복되는 비율 0.5로 선언

    const bool bInitImu = mpAtlas->isImuInitialized();  // Atlas에 IMU 초기화가 되어있는지 bInitImu 변수로 확인
    int count=0;    // KeyFrame의 갯수를 세는 변수

    // Compoute last KF from optimizable window:
    unsigned int last_ID;   // Last KeyFrame ID를 저장할 변수

    if (mbInertial) // IMU를 사용할 때 - Last ID 사용 & count 증가
    {
        int count = 0;
        KeyFrame* aux_KF = mpCurrentKeyFrame;   // aux는 Auxiliary의 줄임말로 추정 (Auxiliary - 보조의)
        while(count<Nd && aux_KF->mPrevKF)  // count가 ND(=21)보다 작고 Prev Key Frame이 존재할 때
        {
            aux_KF = aux_KF->mPrevKF;   // aux_KF를 Prev Key Frame으로 선언
            count++;    // count를 1 증가
        }
        last_ID = aux_KF->mnId; // while문을 빠져 나오면 조건에 포함된 가장 이전의 Key Frame을 Last ID로 선언
    }


    // Local Key Frames를 for문을 활용하여 순회
    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        count++;    // count를 1 증가
        KeyFrame* pKF = *vit;

        if((pKF->mnId==pKF->GetMap()->GetInitKFid()) || pKF->isBad())   // Init Key Frame ID이거나 KeyFrame이 Bad일 경우
            continue;   // Skip
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();    // KeyFrame에서 관찰된 Map Points들을 벡터로 저장

        int nObs = 3;   // Observation 갯수 선언
        const int thObs=nObs;   // Observation Threshold 갯수 선언
        int nRedundantObservations=0;   // 중복되는 Observation을 Count하기 위한 변수
        int nMPs=0; // Map Point의 갯수를 Count하기 위한 변수

        // for문을 활용하여 Key Frame에서 관찰된 Map point 순회
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i]; // 하나의 Map Point를 포인터로 가르킨다.
            if(pMP)
            {
                if(!pMP->isBad())   // Map Point가 Bad가 아니라면
                {
                    if(!mbMonocular)    // Monocular mode가 아닐 경우
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)  //Depth값이 부정확할 때 (Threshold Depth보다 크거나, 0보다 작을 때)
                            continue;   // Skip
                    }

                    nMPs++; // Map Point 갯수 1 증가

                    if(pMP->Observations()>thObs)   
                    // Map Point의 Observation이 (하나의 Map point가 서로 다른 Key Frame에서 관찰되는 수) Threshold보다 클 때
                    {
                        // 현재 Key Frame의 ScaleLevel (이미지에서 scale) 값을 가져온다. 참고 - Scale Space와 이미지 피라미드(image pyramid) [https://www.whydsp.org/247]
                        // 참고 그림 - https://en.wikipedia.org/wiki/Pyramid_(image_processing)#/media/File:Image_pyramid.svg
                        // NLeft는 KeyPoint를 담고 있는 벡터의 갯수
                        // 만역 Key Frmae에서 KeyPoint를 담고 있는 벡터의 갯수가 -1 (없다면) Undistorted Keypoint의 octave를 scaleLevel로 대입
                        //      i(iterator를 돌고 있는 Map point)가 KeyFrame의 NLeft보다 작을 경우 i번째 MapPoint가 관찰된 octave를 저장
                        //          그렇지 않을 경우 mvKeysRight(Right image에서 관찰된 Map point)의 octave를 대입
                        const int &scaleLevel = (pKF -> NLeft == -1) ? pKF->mvKeysUn[i].octave
                                                                     : (i < pKF -> NLeft) ? pKF -> mvKeys[i].octave
                                                                                          : pKF -> mvKeysRight[i].octave;
                        
                        // Current Frame의 Map point를 관찰하고 있는 여러 Keyframe을 observation 변수를 이용하여 저장
                        const map<KeyFrame*, tuple<int,int>> observations = pMP->GetObservations();

                        int nObs=0; // Observation 갯수 선언

                        // 하나의 Map point를 관찰하고 있는 KeyFrame에 대해서 Observation이라는 구조를 활용하여 순회
                        for(map<KeyFrame*, tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;    // Map point를 관찰하고 있는 Key Frame을 pointer로 선언
                            if(pKFi==pKF)   // Line 1030에서 선언한 Key Frame과 같은 Key Frame일 경우
                                continue;   // Skip

                            tuple<int,int> indexes = mit->second;
                            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

                            // Index를 활용하여 ScaleLeveli를 계산
                            // NLeft == -1 일 때, Undistorted Keypoints를 활용
                            // leftIndex != -1 일 때, Keypoints를 활용
                            // rightIndex != -1 일 때, Right Keypoints를 활용
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

                            if(scaleLeveli<=scaleLevel+1)   // ScaleLeveli가 현재 Key Frame의 ScaleLevel + 1보다 이하일 경우
                            {
                                nObs++; // Observation 갯수 1 증가
                                if(nObs>thObs)  // Observation 갯수가 3을 초과할 경우
                                    break;  // Observation에 대한 for문을 빠져나온다.
                            }
                        }


                        if(nObs>thObs)  // Observation 갯수가 3을 초과할 경우
                        {
                            nRedundantObservations++;   // 중복되는 Observation을 Count 1 증가
                        }
                    }
                }
            }
        }   // Map point 순회에 대한 for문에 대한 괄호

        // 중복되는 Observation의 갯수가 중복되는 비율 * Map Point의 갯수를 넘을 때  
        // 증복되는 Key Frame을 제거하는 과정
        if(nRedundantObservations>redundant_th*nMPs)
        {
            if (mbInertial) // IMU를 사용할 경우
            {
                if (mpAtlas->KeyFramesInMap()<=Nd)  // Atlas에 있는 Key Frame의 갯수를 가져와서 Nd(LIBA와 같은 KeyFrame의 갯수)와 비교
                // Atlas에 있는 KeyFrame의 갯수가 Nd보다 더 작거나 같은 경우
                    continue;   // Skip

                if(pKF->mnId > (mpCurrentKeyFrame->mnId-2)) // Key Frame의 ID가 Current Key Frame의 ID -2 보다 큰 경우
                    continue;   // Skip

                if(pKF->mPrevKF && pKF->mNextKF)    // Prev Key Frame과 Next Key Frame이 모두 존재한다면
                {
                    const float t = pKF->mNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;  
                    // Next KeyFrame의 Timestamp와 Prev KeyFrame의 Timestamp의 차이 계산

                    if((bInitImu && (pKF->mnId<last_ID) && t<3.) || (t<0.5))
                    // IMU 초기화를 완료한 상태이고, KeyFrame의 ID가 last_ID보다 작고, Timestamp가 3 미만일 경우이거나,
                    // Timestamp가 0.5 미만일 경우 (Prev KeyFrame과 Next KeyFrame이 너무 차이가 없을 경우)
                    {
                        // KeyFrame 제거
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();  // Key Frame의 Graph와 관련된 값들을 모두 Erase (Weight, Observation, Parent 등등..) 
                    }
                    else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && (cv::norm(pKF->GetImuPosition()-pKF->mPrevKF->GetImuPosition())<0.02) && (t<3))
                    // KeyFrame이 존재하는 Map에 Inertial BA2가 되어 있지 않고,
                    // 현재 IMU position과 이전 IMU position에 대한 Norm값이 0.02미만이고,
                    // Timestamp의 값이 3미만일 경우
                    {
                        // KeyFrame 제거
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();  // Key Frame의 Graph와 관련된 값들을 모두 Erase (Weight, Observation, Parent 등등..) 
                    }
                }
            }
            else    // IMU를 사용하지 않을 경우
            {
                pKF->SetBadFlag();  // Key Frame의 Graph와 관련된 값들을 모두 Erase (Weight, Observation, Parent 등등..) 
            }
        }


        if((count > 20 && mbAbortBA) || count>100)  // count는 Local KeyFrame에 대한 for문을 돌 때마다 1증가 (Line 1029)
        // 20번 이상 KeyFrame에 대한 for문을 돌면서 mbAbortBA값이 true인 경우이거나
        // 100번 이상 KeyFrame에 대한 for문을 돈 경우
        {
            break;  // Local Key Frames를 순회하는 for문을 빠져 나온다.
        }
    }
}


cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    // Input Matrix v의 경우 (3x1)
    // LocalMapping::ComputeF12()에서 사용

    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

cv::Matx33f LocalMapping::SkewSymmetricMatrix_(const cv::Matx31f &v)
{
    // Input Matrix v의 경우 (3x1)
    // LocalMapping::ComputeF12_()에서 사용

    cv::Matx33f skew{0.f, -v(2), v(1),
                     v(2), 0.f, -v(0),
                     -v(1), v(0), 0.f};

    return skew;
}


//^ Local Mapping 3가지 종류의 Control Flag
//^ 1. Finish : Local Mapper의 종료 (-> System에서 Shutdown 호출 시 Local Mapper Finish) 
//^ 2. Stop   : Local Mapper의 Run이 멈추어야 할 때
//^     1) Localization Mode일 때 : Map을 더 생성해야할 필요가 없으므로 Local Mapper 사용 X (Only Tracking)
//^     2) Map에 대한 최적화 진행할 때 
//^       - Corret Loop 일 때  : 전체 Map에 대해서 Loop Closing 시 새로운 Keyframe들이 들어오는 것을 막기 위해
//^       - MergeLocal 일 때   : Local Map merge 한 후 Map에 대해 최적화 할 때 새로운 Keyframe들이 들어오는 것을 막기 위해 
//^       - GlobalBA 실행될 때 : Global bundle adjustment 시 새로운 Keyframe들이 들어오는 것을 막기 위해
//^ 3. Reset  : Local Mapper 초기화
//^     1) Reset : 시스템 처음 실행 시, 1회 호출 (TrackMonocular, TrackStereo, TrackRGBD)
//^     2) ResetActiveMap : 시스템 Running 중에 Local Mapper의 초기화가 필요할 때 마다 호출
//^         - System의 ChangeDataset()이 호출될 때 -> offline SLAM일 때 image sequence 들어올 때마다 
//^         - Tracking에서 TRACK_LOST 될 때 

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
        //^ Tracking의 ResetActiveMap에서 실행
        //^ pMap : Atlas의 CurrentMap 
        //^ LocalMapping에서는 이 pMap이 따로 사용되지는 않음.
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
    //^ Reset이 하는 일
    //^ 1. Inertial parameter 초기화
    //^ 2. mlNewKeyFrames, mlpRecentAddedMapPoints clear
    bool executed_reset = false;
    {
        unique_lock<mutex> lock(mMutexReset);
        //^ mbResetRequested && mbResetRequestedActiveMap 리셋
        //^ Reset이 요청되면 Local Mapping에서는
        //^ IMU Initialize, Scale Refinement를 실행하지 않음 (내부 parameter들 Reset되어야 하므로)
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

            //^ IMU Initialization 횟수 (디버깅용)
            mIdxInit=0;

            cout << "LM: End reseting Local Mapping..." << endl;
        }
        //^ mbResetRequestedActiveMap만 리셋
        //^ ResetActiveMap은 Track Lost일때 진행 된다.
        //^ *** Active Map은 Tracking에서 incoming frames가 localize 하는 데에 사용되고,
        //^ *** Local Mapping에서는 Tracking이 완료된 keyframes가 보정되고 active map에 추가된다.
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
    if (mbResetRequested)   // Map Reset이 Request된 경우
        return;     // InitialzieIMU 수행 끝

    float minTime;  // 최소 TimeStamp 차이 (시간 차이)
    int nMinKF;     // 최소 Key Frame의 갯수

    if (mbMonocular)    // Monocular 모드일 경우
    {
        minTime = 2.0;  // 2s (Unix Time 변환 : https://www.epochconverter.com/)
        nMinKF = 10;
    }
    else    // Stereo 모드일 경우
    {
        minTime = 1.0;  // 1s
        nMinKF = 10;
    }

    if(mpAtlas->KeyFramesInMap()<nMinKF)    // Map에 존재하는 Key Frame의 갯수가 nMinKF보다 작을 경우
        return;    // InitialzieIMU 수행 끝

    // Retrieve all keyframe in temporal order
    list<KeyFrame*> lpKF;   // KeyFrame을 List로 담을 변수 선언
    KeyFrame* pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF) // 이전 KeyFrame이 존재할 때 계속 수행
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end());    // list에 담을 KeyFrame을 Vector 자료구조를 이용해서 담는다.

    if(vpKF.size()<nMinKF)  // vpKF의 갯수가 최소 KeyFrame의 갯수보다 작을 경우
        return;     // InitialzieIMU 수행 끝

    mFirstTs=vpKF.front()->mTimeStamp;  
    // 가장 첫번째 KeyFrame (Current Key Frame에서 가장 먼 TimeStamp를 가진 KeyFrame)의 TimeStamp를 가져온다.

    if(mpCurrentKeyFrame->mTimeStamp-mFirstTs<minTime)  // Current KeyFrame의 TimeStamp와 첫번째 TimeStamp의 차이가 minTime이하 일 경우
    // KeyFrame 간 TimeStamp가 얼마 안 떨어졌을 경우
        return;     // InitialzieIMU 수행 끝

    // bInitializing = true로 바뀌기전에 함수를 빠져 나온다는 것은 InitialzieIMU를 수행하지 못했다는 뜻으로 해석
    // InitializeIMU를 본격적으로 수행

    bInitializing = true;   // Initializing 변수를 true로 바꿈 (초기화중임을 알려주는 변수)

    while(CheckNewKeyFrames())  // 새로운 KeyFrame이 있는지 검사 (mlNewKeyFrames가 empty일 때 까지)
    {
        ProcessNewKeyFrame();   // mlNewKeyFrames의 첫번째 원소를 mpCurrentKeyFrame으로 대입
        vpKF.push_back(mpCurrentKeyFrame);  // mpCurrentKeyFrame을 vpKF에 push_back
        lpKF.push_back(mpCurrentKeyFrame);  // mpCurrentKeyFrame을 lpKF에 push_back
    }

    const int N = vpKF.size();  // KeyFrame의 갯수를 N으로 선언
    IMU::Bias b(0,0,0,0,0,0);   // IMU bias값 초기화

    // Compute and KF velocities mRwg estimation
    if (!mpCurrentKeyFrame->GetMap()->isImuInitialized())
    // Current Key Frame의 Map이 IMU Initialize가 안되어 있을 경우
    {
        // 참고하면 좋은 그림 - 해당 논문 Figure 1 : https://www.mdpi.com/2072-4292/12/18/3048/html

        cv::Mat cvRwg;  // Rotation Matrix (Gravity -> World) - if문 안에서 구하려고 하는 것
        cv::Mat dirG = cv::Mat::zeros(3,1,CV_32F);  // direction Gravity - 월드 좌표계에서 봤을 때 중력의 방향
        for(vector<KeyFrame*>::iterator itKF = vpKF.begin(); itKF!=vpKF.end(); itKF++)  // Key Frame을 for문 이용해서 순회
        {
            if (!(*itKF)->mpImuPreintegrated)   // Imu Preintegrated가 처리 되어 있지 않으면
                continue;
            if (!(*itKF)->mPrevKF)  // 이전 Key Frame이 존재 하지 않으면
                continue;

            dirG -= (*itKF)->mPrevKF->GetImuRotation()*(*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();
            // for문을 활영하여 IMU preintegrated 되어 있는 속도 값을 누적 >> Direction 값을 얻어냄
            // 속도 변화량, -는 중력방향
            cv::Mat _vel = ((*itKF)->GetImuPosition() - (*itKF)->mPrevKF->GetImuPosition())/(*itKF)->mpImuPreintegrated->dT;
            // velocity = (Key Frame의 IMU position과 이전 Key Frame의 IMU position) / 시간의 변화량
            
            // IMU 속도 = Key Frame의 속도
            (*itKF)->SetVelocity(_vel); // Key Frame의 Velocity 설정
            (*itKF)->mPrevKF->SetVelocity(_vel);    // 이전 Key Frame의 Velocity 설정
        }

        // Gravity와 World의 Rotation Matrix를 구하기 위한 과정
        dirG = dirG/cv::norm(dirG); // unit vector로 만들기
        cv::Mat gI = (cv::Mat_<float>(3,1) << 0.0f, 0.0f, -1.0f);   // World Coordinate gravity vector (z방향으로 -1)
        cv::Mat v = gI.cross(dirG);     
        const float nv = cv::norm(v);   
        const float cosg = gI.dot(dirG);    
        const float ang = acos(cosg);   // gI랑 diG가 이루는 Angle (Radian)
        cv::Mat vzg = v*ang/nv; 
        cvRwg = IMU::ExpSO3(vzg);
        mRwg = Converter::toMatrix3d(cvRwg);
        mTinit = mpCurrentKeyFrame->mTimeStamp-mFirstTs;    // TimeStamp 차이를 활용하여 Tinit 계산
    }
    else    //  Current Key Frame의 Map이 IMU Initialize가 되어 있을 경우
    {
        mRwg = Eigen::Matrix3d::Identity(); // 단위행렬 - 회전이 없다
        mbg = Converter::toVector3d(mpCurrentKeyFrame->GetGyroBias());  // Gyro Bias 값을 대입
        mba = Converter::toVector3d(mpCurrentKeyFrame->GetAccBias());   // Acc Bias 값을 대입
    }

    mScale=1.0;

    mInitTime = mpTracker->mLastFrame.mTimeStamp-vpKF.front()->mTimeStamp;
    // 초기화함수 선언 (딱히 쓰이는 곳은 없음)

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();    // 현재 시간 측정
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg, mba, mbMonocular, infoInertial, false, false, priorG, priorA);
    // Optimizer::InertialOptimization() 중 첫번째 함수
    // Inertial Optimization으로 Key Frame의 Pose, KeyFrame에 새로운 velocities and IMU biases, Scale값도 다시 계산
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();    // 현재 시간 측정 

    if (mScale<1e-1)    // mScale값이 1/10 이하면
    {
        cout << "scale too small" << endl;
        bInitializing=false;    // InitializeIMU 수행 끝
        return; // InitializeIMU 함수 반환
    }

    // Before this line we are not changing the map
    // 다음 Line부터 IMU Initialize를 통해 map point들이 수정될 수 있다.

    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();    // 현재 시간 측정 
    if ((fabs(mScale-1.f)>0.00001)||!mbMonocular)   // fabs()는 절대값 구하는 함수
    // Scale 값이 1.0 근처가 아니거나 Stereo Mode, Stereo-Inertial 모드일 경우
    {
        // Updated Scale값을 Atlas Map에 적용
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Converter::toCvMat(mRwg).t(),mScale,true);
        // Updated Scale값을 Tracking에 적용
        mpTracker->UpdateFrameIMU(mScale,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();    // 현재 시간 측정 

    // Check if initialization OK
    // IMU Initialization에 대한 bool type을 true로 바꿔줌 (IMU Initialization을 수행 중이므로)
    if (!mpAtlas->isImuInitialized()) // Atlas에 IMU 초기화가 되어 있지 않다면
        for(int i=0;i<N;i++)    // KeyFrame의 갯수만큼 for문
        {
            KeyFrame* pKF2 = vpKF[i];
            pKF2->bImu = true;  // KeyFrame마다 IMU에 대한 bool type을 true로 바꿔줌
        }

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();    // 현재 시간 측정 
    if (bFIBA)  // FIBA - Full Inertial BA의 줄임말 
    {
        if (priorA!=0.f)    // 이전 Acc의 값이 0이 아니라면
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, 0, NULL, true, priorG, priorA);
            // Full Inertial Bundle Adjustment로 Camera pose, Camera velocity, Map point, IMU bias 값을 Update
        else    // 이전 Acc의 값이 0이면
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, 0, NULL, false);
            // Full Inertial Bundle Adjustment로 Camera pose, Camera velocity, Map point, IMU bias 값을 Update
    }

    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();    // 현재 시간 측정 

    // If initialization is OK - 초기화를 모두 진행 했다면

    mpTracker->UpdateFrameIMU(1.0,vpKF[0]->GetImuBias(),mpCurrentKeyFrame); 
    // Tracking의 IMU Frame을 Current Key Frame으로 update

    if (!mpAtlas->isImuInitialized())   // Atlas에 IMU 초기화가 되어 있지 않다면
    {
        cout << "IMU in Map " << mpAtlas->GetCurrentMap()->GetId() << " is initialized" << endl;
        // Current Map의 ID를 활용하여 IMU 초기화를 진행
        mpAtlas->SetImuInitialized();
        // Atlas에 IMU 초기화를 진행
        mpTracker->t0IMU = mpTracker->mCurrentFrame.mTimeStamp; 
        // time-stamp of IMU initialization를 Current Frame의 TimeStamp를 활용
        mpCurrentKeyFrame->bImu = true;
        // Current Key Frame의 IMU에 대한 변수를 true로 선언
    }

    mbNewInit=true;     // 새로운 Init이라는 것을 알려주는 변수 (딱히 쓰이지는 않음)
    mnKFs=vpKF.size();  // KeyFrame의 갯수를 vpKF의 갯수를 대입
    mIdxInit++; // 초기화 진행 Index Update

    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();   // Key Frame을 제거하기전 KeyFrame에 대한 Graph 관계에 대한 초기화 과정을 진행 - 삭제를 하므로
        delete *lit;    // KeyFrame을 삭제
    }

    mlNewKeyFrames.clear(); // mlNewKeyFrames Clear로 초기화

    mpTracker->mState=Tracking::OK;     // Tracking의 현재 state를 OK로 바꿈 
    bInitializing = false;  // InitializeIMU 수행 끝 

    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex(); 
    // Current Key Frame의 Map이 바뀌었다는 알려주는 mnMapChange 변수 Index를 1 증가

    return; // InitializeIMU 함수 반환
}

void LocalMapping::ScaleRefinement()
{
     // Minimum number of keyframes to compute a solution
    // Minimum time (seconds) between first and last keyframe to compute a solution. Make the difference between monocular and stereo
    // unique_lock<mutex> lock0(mMutexImuInit);
    if (mbResetRequested) //reset requested가 true일때 return합니다.
        return;

    // Retrieve all keyframes in temporal order
    list<KeyFrame*> lpKF; //keyframe list를 생성합니다.
    KeyFrame* pKF = mpCurrentKeyFrame; //keyframe을 currentkeyframe에서 가져옵니다.
    while(pKF->mPrevKF) //while문 시작!
    {
        lpKF.push_front(pKF); //lpKF 리스트에 해당 pKF를 입력합니다.
        pKF = pKF->mPrevKF; //현재 keyframe기준으로 계속해서 이전 keyframe을 가져옵니다.
    }

    //while문이 끝나면서 리스트에는 current keyframe기준 받아왔던 previous key frame들을 모두 저장하게됩니다.
    
    lpKF.push_front(pKF);  //마지막 previous keyframe을 넣습니다.
    //lpKF 리스트 업데이트 완료

    vector<KeyFrame*> vpKF(lpKF.begin(),lpKF.end()); //vpKF 벡터를 초기화하여 만듭니다. begin end는 string값을 반환하며 vector 생성에서 크기 및 초기값을 정해줍니다.

    while(CheckNewKeyFrames()) //while문이 돌아갑니다. 해당 while문은 새로운 keyframe이 들어왔을때 진행됩니다.
    {
        ProcessNewKeyFrame(); //new keyframe 기본작업을 합니다. 여기서 current keyframe이 update됩니다.
        vpKF.push_back(mpCurrentKeyFrame); //update된 current keyframe을 각각 리스트, 벡터에 대입합니다.
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size(); //vector사이즈 정의 --> 위에서 update된 keyframe갯수에 따라 달라집니다. 

    mRwg = Eigen::Matrix3d::Identity(); //3x3 단위행렬을 생성합니다.
    mScale=1.0; //mScale을 1.0으로 초기화합니다. 

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now(); //최적화 시작지점 기록
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale); //초기값으로부터 optimizer 시작 (해당부분은 scale값과 중력방향만 최적합니다.)
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now(); //optimizer 종료지점 기록

    if (mScale<1e-1) //optimizer 과정에서 update된 scale값이 1e-1보다 작으면 실행됩니다. 
    {
        cout << "scale too small" << endl; //scale이 너무 작아서 initializing 실패
        bInitializing=false;
        return;
    }
    //해당 1e-1보다 크게되면 initializing이 재대로 되었다고 판단하고 사용한다.

    // Before this line we are not changing the map
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    if ((fabs(mScale-1.f)>0.00001)||!mbMonocular) //해당 조건을 만족하면 업데이트를 시작합니다.
    {
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Converter::toCvMat(mRwg).t(),mScale,true); //최적화된 mscale값과 rwg값을 넣습니다. 
        mpTracker->UpdateFrameIMU(mScale,mpCurrentKeyFrame->GetImuBias(),mpCurrentKeyFrame); //최적화된 pose, imu데이터, 맵포인트를 업데이트합니다.
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now(); //업데이트 종료시점 기록

    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++) //해당과정에서 setbadflag가 존재하는 keyframe은 제거합니다.
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }

    mlNewKeyFrames.clear(); //new keyframe 초기화

    double t_inertial_only = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count(); //inertial only 최적화 과정에 소요된 시간 계산

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
