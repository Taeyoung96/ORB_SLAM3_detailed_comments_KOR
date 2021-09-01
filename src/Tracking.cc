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


#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Initializer.h"
#include "G2oTypes.h"
#include "Optimizer.h"

#include <iostream>

#include <mutex>
#include <chrono>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>
#include <include/MLPnPsolver.h>

using namespace std;

namespace ORB_SLAM3
{

/* system, orbvocabulary, framedrawer, mapdrawer, atlas, keyframedatabase --> 각각의 class들을 포인터로 선언, 나중에 tracking중에 해당 클래스의 변수들을 가져올때 대부분 사용한다.

strSettingPath, sensor, nameSeq --> 상수로 선언함으로써 나중에 고정변수로 사용한다.

Tracking states를 나타내는 변수  NO_IMAGES_YET=0

int mSensor --> 사용할 센서 (image , imu ...)

int mTrackedFr, bool mbStep --> tracking frame을 진행하고 result를 통한 pose estimate (bool 타입)

bool mbOnlyTracking; --> True if local mapping is deactivated and we are performing only localization 

bool mbVO -->     // In case of performing only localization, this flag is true when there are no matches to
  points in the map. Still tracking will continue if there are enough matches with temporal points.
  In that case we are doing visual odometry. The system will try to do relocalization to recover
  "zero-drift" localization to the map. 해석해보면 localization모드일때(local mapping x), 만약 맵에서 point들과 match가 이루어지지 않는다면 true로 된다. 이게 아니라 만약에 순간적인 point들과 match가 충분히 이루어진다면 tracking은 계속될것이다. 
  그렇게되면 우리는 visiual odometry를 진행할수 있다. (시각적인 경로탐지?정도로 해석가능할듯) 그러고나면 정상적으로 relocalization이 이루어지고 drift를 최소화시킨다.

mpORBVocabulary, mpKeyFrameDB --> Bag of words

mpInitializer --> monocular에서만 필요함. 

mpSystem --> system 멤버변수

mpViewer, mpFrameDrawer ,mpMapDrawer  --> map viewer에 사용

mpAtlas

mnLastRelocFrameId, time_recently_lost, time_recently_lost_visual, mnInitialFrameId, mnFirstFrameId --> Last Frame, KeyFrame and Relocalisation Info

mbCreatedMap

mpCamera2 
*/

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, const string &_nameSeq):
    mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
    mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
    mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0), time_recently_lost_visual(2.0),
    mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr)
{
    //해당 함수는 tracking을 하기 위한 초기값 세팅의 시작으로 볼수 있습니다.

    // Load camera parameters from settings file
    //config 파일에서 fx,fy,cx,cy등등을 가져옵니다. 해당 config파일은 example파일에 예시가 있습니다. 
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool b_parse_cam = ParseCamParamFile(fSettings); //config파일에서 불러온 parameter들로 parsecamparamfile함수에서 연산을 합니다. 
    if(!b_parse_cam) //위의 bool타입 함수, 즉 camera parameter가 잘 들어왔는지 확인하는작업입니다. 
    {
        std::cout << "*Error with the camera parameters in the config file*" << std::endl; //에러메세지
    }

    // Load ORB parameters
    //camera parameter와 마찬가지로 ORB parameter를 불러와서 연산을 합니다. 해당 parameter들도 example파일에서 .yaml파일에 보면 나와있습니다.
    bool b_parse_orb = ParseORBParamFile(fSettings);
    if(!b_parse_orb) //camera 부분과 마찬가지입니다. 
    {
        std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
    }

    initID = 0; lastID = 0; //초기값 선언

    // Load IMU parameters
    bool b_parse_imu = true; //초기 bool값 true선언 
    if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO) //IMU MONOCULAR OR IMU STEREO 일때만 실행됩니다. 
    {
        b_parse_imu = ParseIMUParamFile(fSettings); //IMU parameter 값 세팅 
        if(!b_parse_imu) //parameter가 재대로 입력되었나 안되었나 판단합니다.
        {
            std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
        }

        //MaxFrame값을 FrameToResetIMU값으로 받는데 이 값은 fps값으로 결정됩니다. 
        //예를 들어 imu fps가 200hz면 200값으로 설정되며 이 값은 각 프레임별로 기준값 계산에 사용됩니다.
        mnFramesToResetIMU = mMaxFrames;
    }

    mbInitWith3KFs = false; //?? 아마 사용이 안되는 변수인걸로 예상합니다. ORB SLAM2 code의 잔해(?)같습니다. 

    mnNumDataset = 0; //dataset number 초기화

    if(!b_parse_cam || !b_parse_orb || !b_parse_imu) //cam, orb, imu에 대한 parsing이 재대로 이루어졌는지 체크합니다. 
    {
        std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
        
        //try, catch 구문이 재대로 작성되지 않았습니다. 아직 수정해야되는 부분이라고 생각합니다. 
        try
        {
            throw -1;
        }
        catch(exception &e) 
        {

        }
    }

#ifdef REGISTER_TIMES
    vdRectStereo_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();

    vdUpdatedLM_ms.clear();
    vdSearchLP_ms.clear();
    vdPoseOpt_ms.clear();
#endif

    vnKeyFramesLM.clear(); //keyframe vector clear
    vnMapPointsLM.clear(); //map points vector clear
}

#ifdef REGISTER_TIMES
double calcAverage(vector<double> v_times)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_values)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(vector<int> v_values, double average)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

void Tracking::LocalMapStats2File()
{
    ofstream f;
    f.open("LocalMapTimeStats.txt");
    f << fixed << setprecision(6);
    f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF culling[ms], Total[ms]" << endl;
    for(int i=0; i<mpLocalMapper->vdLMTotal_ms.size(); ++i)
    {
        f << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCulling_ms[i] << ","
          << mpLocalMapper->vdMPCreation_ms[i] << "," << mpLocalMapper->vdLBA_ms[i] << ","
          << mpLocalMapper->vdKFCulling_ms[i] <<  "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
    }

    f.close();

    f.open("LBA_Stats.txt");
    f << fixed << setprecision(6);
    f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
    for(int i=0; i<mpLocalMapper->vdLBASync_ms.size(); ++i)
    {
        f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i] << ","
          << mpLocalMapper->vnLBA_KFfixed[i] << "," << mpLocalMapper->vnLBA_MPs[i] << ","
          << mpLocalMapper->vnLBA_edges[i] << endl;
    }


    f.close();
}

void Tracking::TrackStats2File()
{
    ofstream f;
    f.open("SessionInfo.txt");
    f << fixed;
    f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
    f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

    f << "OpenCV version: " << CV_VERSION << endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << fixed << setprecision(6);

    f << "#KF insert[ms], ORB ext[ms], Stereo match[ms], IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;

    for(int i=0; i<vdTrackTotal_ms.size(); ++i)
    {
        double stereo_rect = 0.0;
        if(!vdRectStereo_ms.empty())
        {
            stereo_rect = vdStereoMatch_ms[i];
        }

        double stereo_match = 0.0;
        if(!vdStereoMatch_ms.empty())
        {
            stereo_match = vdStereoMatch_ms[i];
        }

        double imu_preint = 0.0;
        if(!vdIMUInteg_ms.empty())
        {
            imu_preint = vdIMUInteg_ms[i];
        }

        f << stereo_rect << "," << vdORBExtract_ms[i] << "," << stereo_match << "," << imu_preint << ","
          << vdPosePred_ms[i] <<  "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i] << endl;
    }

    f.close();

    f.open("TrackLocalMapStats.txt");
    f << fixed << setprecision(6);

    f << "# number of KF, number of MP, UpdateLM[ms], SearchLP[ms], PoseOpt[ms]" << endl;

    for(int i=0; i<vnKeyFramesLM.size(); ++i)
    {

        f << vnKeyFramesLM[i] << "," << vnMapPointsLM[i] <<  "," << vdUpdatedLM_ms[i] << "," << vdSearchLP_ms[i] << "," << vdPoseOpt_ms[i] << endl;
    }

    f.close();
}

void Tracking::PrintTimeStats()
{
    // Save data in files
    TrackStats2File();
    LocalMapStats2File();


    ofstream f;
    f.open("ExecTimeMean.txt");
    f << fixed;
    //Report the mean and std of each one
    std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    cout << "OpenCV version: " << CV_VERSION << endl;
    f << "OpenCV version: " << CV_VERSION << endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    f << "---------------------------" << std::endl;
    f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;
    if(!vdRectStereo_ms.empty())
    {
        average = calcAverage(vdRectStereo_ms);
        deviation = calcDeviation(vdRectStereo_ms, average);
        std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdORBExtract_ms);
    deviation = calcDeviation(vdORBExtract_ms, average);
    std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
    f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

    if(!vdStereoMatch_ms.empty())
    {
        average = calcAverage(vdStereoMatch_ms);
        deviation = calcDeviation(vdStereoMatch_ms, average);
        std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(!vdIMUInteg_ms.empty())
    {
        average = calcAverage(vdIMUInteg_ms);
        deviation = calcDeviation(vdIMUInteg_ms, average);
        std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
        f << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdPosePred_ms);
    deviation = calcDeviation(vdPosePred_ms, average);
    std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;
    f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdLMTrack_ms);
    deviation = calcDeviation(vdLMTrack_ms, average);
    std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
    f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdNewKF_ms);
    deviation = calcDeviation(vdNewKF_ms, average);
    std::cout << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;
    f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdTrackTotal_ms);
    deviation = calcDeviation(vdTrackTotal_ms, average);
    std::cout << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

    // Local Map Tracking complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Local Map Tracking complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Local Map Tracking complexity (mean$\\pm$std)" << std::endl;

    average = calcAverage(vnKeyFramesLM);
    deviation = calcDeviation(vnKeyFramesLM, average);
    std::cout << "Local KFs: " << average << "$\\pm$" << deviation << std::endl;
    f << "Local KFs: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vnMapPointsLM);
    deviation = calcDeviation(vnMapPointsLM, average);
    std::cout << "Local MPs: " << average << "$\\pm$" << deviation << std::endl;
    f << "Local MPs: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Local Mapping" << std::endl << std::endl;
    f << std::endl << "Local Mapping" << std::endl << std::endl;

    average = calcAverage(mpLocalMapper->vdKFInsert_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
    std::cout << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
    std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCreation_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
    std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLBASync_ms);
    deviation = calcDeviation(mpLocalMapper->vdLBASync_ms, average);
    std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdKFCullingSync_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFCullingSync_ms, average);
    std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLMTotal_ms);
    deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
    std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping LBA complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_edges);
    deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
    std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFopt);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
    std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
    std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_MPs);
    deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
    std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
    f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

    // Map complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Map complexity" << std::endl;
    std::cout << "KFs in map: " << mpAtlas->GetAllMaps()[0]->GetAllKeyFrames().size() << std::endl;
    std::cout << "MPs in map: " << mpAtlas->GetAllMaps()[0]->GetAllMapPoints().size() << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    f << "KFs in map: " << mpAtlas->GetAllMaps()[0]->GetAllKeyFrames().size() << std::endl;
    f << "MPs in map: " << mpAtlas->GetAllMaps()[0]->GetAllMapPoints().size() << std::endl;


    // Place recognition time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Place Recognition (mean$\\pm$std)" << std::endl << std::endl;
    f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl << std::endl;

    average = calcAverage(mpLoopClosing->vTimeBoW_ms);
    deviation = calcDeviation(mpLoopClosing->vTimeBoW_ms, average);
    std::cout << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vTimeSE3_ms);
    deviation = calcDeviation(mpLoopClosing->vTimeSE3_ms, average);
    std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vTimePRTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vTimePRTotal_ms, average);
    std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl;

    // Loop Closing time stats
    if(mpLoopClosing->vTimeLoopTotal_ms.size() > 0)
    {
        std::cout << std::endl << std::endl << std::endl;
        std::cout << "Loop Closing (mean$\\pm$std)" << std::endl << std::endl;
        f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl << std::endl;

        average = calcAverage(mpLoopClosing->vTimeLoopTotal_ms);
        deviation = calcDeviation(mpLoopClosing->vTimeLoopTotal_ms, average);
        std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl;
        f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(mpLoopClosing->vTimeMergeTotal_ms.size() > 0)
    {
        // Map Merging time stats
        std::cout << std::endl << std::endl << std::endl;
        std::cout << "Map Merging (mean$\\pm$std)" << std::endl << std::endl;
        f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl << std::endl;

        average = calcAverage(mpLoopClosing->vTimeMergeTotal_ms);
        deviation = calcDeviation(mpLoopClosing->vTimeMergeTotal_ms, average);
        std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl;
        f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl;
    }


    if(mpLoopClosing->vTimeGBATotal_ms.size() > 0)
    {
        // Full GBA time stats
        std::cout << std::endl << std::endl << std::endl;
        std::cout << "Full GBA (mean$\\pm$std)" << std::endl << std::endl;
        f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl << std::endl;

        average = calcAverage(mpLoopClosing->vTimeFullGBA_ms);
        deviation = calcDeviation(mpLoopClosing->vTimeFullGBA_ms, average);
        std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
        f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vTimeMapUpdate_ms);
        deviation = calcDeviation(mpLoopClosing->vTimeMapUpdate_ms, average);
        std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
        f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
        average = calcAverage(mpLoopClosing->vTimeGBATotal_ms);
        deviation = calcDeviation(mpLoopClosing->vTimeGBATotal_ms, average);
        std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl;
        f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl;
    }

    f.close();

}

#endif

Tracking::~Tracking()
{

}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings) //cam parameter들을 parsing하는 함수입니다. 
{
    mDistCoef = cv::Mat::zeros(4,1,CV_32F); //4x1 사이즈 zero matrix 생성
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false; //b_miss_params 값 false 선언

    string sCameraName = fSettings["Camera.type"]; //config 파일에서 camera type값 가져옵니다.
    if(sCameraName == "PinHole") //sCameraName값이 PinHole일 경우
    {
        float fx, fy, cx, cy; // fx,fy는 초점거리 , cx, cy는 주점을 뜻합니다.

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"]; //node에 camera.fx 값을 가져옵니다.
        if(!node.empty() && node.isReal()) //node에 값이 비어있지 않고 부동소수점값이면 실행합니다.
        {
            fx = node.real(); //해당값을 가져옵니다. 
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;  //b_miss_params값 true
        }

        //이하 앞의 fx의 형식과 모두 동일합니다.
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters 
        //k1,k2: radial distortion 계수, p1,p2: tangential distortion 계수
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real(); //초기에 설정해놓은 4x1 매트릭스에 대입합니다.
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        //해당값은 어떤건지 아직 모르겠습니다.코드상으로는 k3값이 주어져있다면 초기의 Distortion 매트릭스의 size를 늘리고 추가해줍니다. 
        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        if(b_miss_params) //위의 k3값 제외하고 한개라도 값이 error가 있을경우 false합니다. 
        {
            return false;
        }

        vector<float> vCamCalib{fx,fy,cx,cy}; //vertor를 생성해줍니다. 

        mpCamera = new Pinhole(vCamCalib); //CameraModel의 Pinhole class를 새로 생성합니다. 

        mpAtlas->AddCamera(mpCamera); //atlas 데이터저장소에 camera를 add합니다.


        //위에서 가져왔던 camera parameter들을 print합니다.
        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


        std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if(mDistCoef.rows==5) //k3값이 있을경우 실행됩니다. 
            std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        //homogeneouse matrix로 생성
        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = fx;
        mK.at<float>(1,1) = fy;
        mK.at<float>(0,2) = cx;
        mK.at<float>(1,2) = cy;

    }
    else if(sCameraName == "KannalaBrandt8") //fisheye camera 모델로 추정됩니다. 
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;

        // Camera calibration parameters
        // 앞서 진행했던 if문과 구조가 동일합니다. 
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        if(!b_miss_params)
        {
            vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
            mpCamera = new KannalaBrandt8(vCamCalib);

            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;
        }

        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO){ //fisheye카메라일때 stereo와 imu stereo일때 실행합니다. 
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            
            // left camera lapping

            int leftLappingBegin = -1; //lapping begin은 left camera의 lapping이 시작되는 왼쪽 column 입니다
            int leftLappingEnd = -1; //laapping end는 left camera의 lapping이 끝나는 오른쪽 column입니다. 

            // right camera lapping
            int rightLappingBegin = -1; 
            int rightLappingEnd = -1; 

            node = fSettings["Camera.lappingBegin"]; //fisheye camera의 lapping area를 설정합니다. 
            if(!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }
            
            //Tlr은 transfomation matrix이고 이건 left camera 와 right camera사이의 transformation matrix입니다. 
            node = fSettings["Tlr"];
            if(!node.empty())
            {
                mTlr = node.mat();
                if(mTlr.rows != 3 || mTlr.cols != 4)
                {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if(!b_miss_params) //b_miss_params가 false이면 실행된다. 즉 모든 param이 정상일때 실행됩니다. 
            {
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                mpFrameDrawer->both = true;

                vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);

                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << mTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if(b_miss_params)
        {
            return false;
        }

        mpAtlas->AddCamera(mpCamera);
        mpAtlas->AddCamera(mpCamera2);
    }


    else //pinhole과 fisheye 둘다 아니라면 실행됩니다. 
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl; //error message
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl; //error message
    }

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
    {
        cv::FileNode node = fSettings["Camera.bf"]; //baseline과 FOV를 통해 bf값을 구할수 있습니다. 
        if(!node.empty() && node.isReal())
        {
            mbf = node.real();
        }
        else
        {
            std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    float fps = fSettings["Camera.fps"]; //camera fps설정값을 가져옵니다 
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0; //minframese는 0값 선언
    mMaxFrames = fps; //maxframes는 해당 fps로 선언됩니다. 

    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"]; //camera image가 rgb인지 bgr인지 입력합니다. 
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO)
    {
        float fx = mpCamera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if(!node.empty()  && node.isReal())
        {
            mThDepth = node.real();
            mThDepth = mbf*mThDepth/fx; //원거리 근거리 임계점을 설정합니다. 
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }
        else
        {
            std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }


    }

    if(mSensor==System::RGBD)
    {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if(!node.empty() && node.isReal())
        {
            mDepthMapFactor = node.real();
            if(fabs(mDepthMapFactor)<1e-5)
                mDepthMapFactor=1;
            else
                mDepthMapFactor = 1.0f/mDepthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true; //error message
        }

    }

    if(b_miss_params) //error message 발생시 false 반환
    {
        return false;
    }

    return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings) //ORB parameter들을 parsing하는 함수입니다.
{
    bool b_miss_params = false; //초기error값 선언
    int nFeatures, nLevels, fIniThFAST, fMinThFAST; //feature 갯수 , nlevel 기준, init fast feature 값, min fast feature 값
    float fScaleFactor; //nlevel에 따른 scale 값


    //특별한건 없습니다. config파일에서 해당하는 param의 값들을 불러옵니다. 
    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings) //IMU paramter들을 parsing하는 함수입니다. 
{
    bool b_miss_params = false;

    //이 함수 또한 마찬가지로 config파일에서 해당하는 param값들을 불러와 저장합니다. 

    cv::Mat Tbc;
    cv::FileNode node = fSettings["Tbc"]; //transformation matrix입니다. body to imu입니다. 
    if(!node.empty())
    {
        Tbc = node.mat();
        if(Tbc.rows != 4 || Tbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }

    cout << endl;

    cout << "Left camera to Imu Transform (Tbc): " << endl << Tbc << endl;

    float freq, Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"]; //imu hz를 불러옵니다. 
    if(!node.empty() && node.isInt())
    {
        freq = node.operator int();
    }
    else
    {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"]; //imu gyro의 noise값입니다 .
    if(!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    const float sf = sqrt(freq);
    cout << endl;
    cout << "IMU frequency: " << freq << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);


    return true;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper; // Localmapping.cc 포인터 클래스 선언
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;    // Loopclosing.cc 포인터 클래스 선언
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;   // Viewer.cc 포인터 클래스 선언 
}

void Tracking::SetStepByStep(bool bSet)
{
    bStepByStep = bSet;   // bool 타입 변수 선언
}



cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename) 
//imRectleft 왼쪽 이미지, imrectright 오른쪽 이미지, timestamp, filename 선언 --> stereo image data를 불러옴
{
    mImGray = imRectLeft;   //left image를 가져옵니다. 
    cv::Mat imGrayRight = imRectRight; //right image를 가져옵니다. 
    mImRight = imRectRight; //right image를 가져옵니다. 

    if(mImGray.channels()==3) //image가 channel이 3개면, 즉 color image data면 실행됩니다. 
    {
        if(mbRGB) //rgb 데이터라면 
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);   //image를 gray scale로 변환합니다. 
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGB2GRAY); //마찬가지로 gray scale로 변환합니다. 
        }
        else //rgb가 아닌 bgr일때
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY); //image를 gray scale로 변환합니다. 
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGR2GRAY); //image를 gray scale로 변환합니다. 
        }
    }
    else if(mImGray.channels()==4)  //image가 rgba일때 --> alpha값이 추가된 데이터 --> 각 픽셀의 투명도를 뜻함. 딱히...
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY); //마찬가지로 모두 gray scale로 변환
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGRA2GRAY);
        }
    }

    if (mSensor == System::STEREO && !mpCamera2) //stereo이고 fisheye가 아닐때를 의미합니다. 
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
    else if(mSensor == System::STEREO && mpCamera2) //stereo이고 fisheye일때를 의미합니다. --> 차이점은 mpCamera2가 들어갑니다. 즉 lapping 포인트를 고려하느냐 안하느냐의 차이점입니다. 
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr);
    else if(mSensor == System::IMU_STEREO && !mpCamera2) //imu stereo이고 pinhole 일때를 의미합니다. 
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib); //추가적인 parameter는 lastframe과 imuclib가 있습니다. 해당기능은 위에서 설명했습니다. 
    else if(mSensor == System::IMU_STEREO && mpCamera2) //imu stereo이고 fisheye일때를 의미합니다. 
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr,&mLastFrame,*mpImuCalib);

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
    vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename) //해당 study에서는 stereo만 진행하고 있으므로 이외의 type은 제외합니다. 
{
    
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)//해당 study에서는 stereo만 진행하고 있으므로 이외의 type은 제외합니다. 
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if (mSensor == System::MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }
    else if(mSensor == System::IMU_MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
        }
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
    }

    if (mState==NO_IMAGES_YET)
        t0=timestamp;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    lastID = mCurrentFrame.mnId;
    Track();

    return mCurrentFrame.mTcw.clone();
}


void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
{
    unique_lock<mutex> lock(mMutexImuQueue); //mutex로 선언된 mMutexImuQueue를 lock하고
    mlQueueImuData.push_back(imuMeasurement); //mlQueueImuData에 새로운 imuMeasurement를 입력합니다. 
}

void Tracking::PreintegrateIMU()
{
    //cout << "start preintegration" << endl;

    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL); //prevframe이 존재하지 않을때 즉. 맨처음 시작임을 뜻합니다. 
        mCurrentFrame.setIntegrated(); //새로운 mutex imu를 선언하게되고 imu preinterate를 true값으로 반환합니다.
        return;
    }

    mvImuFromLastFrame.clear(); //preframe이 존재하고 lastframe에 있던 imu data를 clear합니다. 
    mvImuFromLastFrame.reserve(mlQueueImuData.size()); //memory size를 기존 queue size만큼 늘려줍니다. 
    if(mlQueueImuData.size() == 0) //만약 queue size가 0일때 실행합니다. 
    {
        Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL); //queue size가 0이라는 의미는 imu데이터가 존재하지 않는다는 의미입니다. 
        mCurrentFrame.setIntegrated(); //imu pre integrated 값을 true로 반환하게 됩니다. 즉 setIntegrated 함수가 실행되면 imu값을 받지 않게됩니다. 
        return;
    }

    while(true) //while문 start
    {
        bool bSleep = false; //초기값 선언
        {
            unique_lock<mutex> lock(mMutexImuQueue); //imu queue를 생성합니다.
            if(!mlQueueImuData.empty()) //imu 데이터가 존재하는 경우
            {
                IMU::Point* m = &mlQueueImuData.front(); //queue의 가장 오래된 imu데이터 주소값을 가져옵니다. 
                cout.precision(17); //소수점자리수를 17개로 설정
                if(m->t < mCurrentFrame.mpPrevFrame->mTimeStamp-0.001l) // 가장 오래된 imu데이터에서 t값(time_stamp)을 가져온뒤 바로 이전 frame의 time stamp -0.001과 비교합니다. 만약 더 크다면 prev frame보다 최신의 imu 데이터라고 볼수 있습니다. 
                {
                    mlQueueImuData.pop_front(); //front의 imu를 삭제합니다. 
                }
                else if(m->t < mCurrentFrame.mTimeStamp-0.001l)  //prev frame보다는 최신데이터이게되면 current frame과 비교합니다. imu데이터들을 저장한뒤 버리고를 반복합니다. 
                {
                    mvImuFromLastFrame.push_back(*m); 
                    mlQueueImuData.pop_front();
                }
                else
                {
                    mvImuFromLastFrame.push_back(*m); //가장 최신의 imu들은 저장합니다. 
                    break;
                }
            }
            else //imu데이터가 존재하지 않는다면 
            {
                break;
                bSleep = true;
            }
        }
        if(bSleep)
            usleep(500); //500milli sec
    }


        const int n = mvImuFromLastFrame.size()-1; //n을 from last frame size의 -1로 선언합니다. 이유는 vector로 선언했기때문입니다. 
    IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,mCurrentFrame.mImuCalib); //Preintegration of Imu Measurements 선언

    for(int i=0; i<n; i++) //lastframe에 있는 imu 데이터를 모두 수행합니다. 
    {
        float tstep; //tstep float형태로 선언합니다.
        cv::Point3f acc, angVel;  //point 형태로 acc, angVel 선언합니다. 
        if((i==0) && (i<(n-1)))  //i가 0이고 n-1보다 작을때, 즉 lastframe에 포함된 imu가 갯수가 1이상일때의 초기값을 의미합니다. 
        {
            float tab = mvImuFromLastFrame[i+1].t - mvImuFromLastFrame[i].t; // 각각 i번째 imu 데이터의 timestamp값을 빼줍니다. 즉 imu 데이터의 시간 간격을 tab에 저장합니다. 
            float tini = mvImuFromLastFrame[i].t - mCurrentFrame.mpPrevFrame->mTimeStamp; //바로 직전 imu데이터의 stamp와 lastframe의 i번째 imu데이터와 시간 간격을 tini에 저장합니다. 
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i+1].a -  (mvImuFromLastFrame[i+1].a - mvImuFromLastFrame[i].a)*(tini/tab))*0.5f; //이부분 같은 경우는 맨 처음부분인데 해당부분은 frame과 정확한 매칭이 어려워서 
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i+1].w - (mvImuFromLastFrame[i+1].w - mvImuFromLastFrame[i].w)*(tini/tab))*0.5f; //medium 적분에 보정값을 넣어줍니다. --> (mvImuFromLastFrame[i+1].w - mvImuFromLastFrame[i].w)*(tini/tab)
            tstep = mvImuFromLastFrame[i+1].t - mCurrentFrame.mpPrevFrame -> mTimeStamp; //tstep은 tini 다음 step을 의미합니다. --> 음수값일텐데....?
        }    
        else if(i<(n-1))
        {
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a)*0.5f; //일반 medium 적분입니다. 
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w)*0.5f; //동일합니다. 
            tstep = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t; //다음 step의 time을 의미합니다. 
        }
        else if((i>0) && (i==(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t; //time offset 값입니다. 
            float tend = mvImuFromLastFrame[i+1].t-mCurrentFrame.mTimeStamp; // 마지막의 imu데이터와 가장 최신의 frame의 시간을 뺀 값입니다. 
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tend/tab))*0.5f; // 맨 처음부분과 동일하게 마지막부분도 frame과의 timestamp가 동일하게 맞기 힘듭니다. 따라서 
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w- // 보정값을 넣어줍니다. 
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tend/tab))*0.5f;
            tstep = mCurrentFrame.mTimeStamp-mvImuFromLastFrame[i].t; 
        }
        else if((i==0) && (i==(n-1)))
        {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }

        if (!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        mpImuPreintegratedFromLastKF-> (acc,angVel,tstep); //해당 acc, angVel, tstep을 mpImuPreintegratedFromLastKF에 저장합니다. 
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep); //직전의 position을 저장한 뒤, Update delta rotation,
        //Compute rotation parts of matrices A and B, Update rotation jacobian wrt bias correction, Total integrated time 작업을 합니다. 
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame; // CurrentFrame class의 imupreintegratedFrame pointer에다가 pImuPreintegratedFromLastFrame을 저장합니다.
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF; // CurrentFrame class의 mpImuPreintegrated pointer에다가 mpImuPreintegratedFromLastKF을 저장합니다.
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame; //마찬가지입니다!

    mCurrentFrame.setIntegrated();  //imu 데이터 정리가 끝났으므로 lock 작업을 하게됩니다. imu를 더이상 받지 않는다라는 뜻으로 이해해도 괜찮습니다. 

    Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG); //preintegration finish
}


bool Tracking::PredictStateIMU()
{
    if(!mCurrentFrame.mpPrevFrame)  //CurrentFrame에서 previous Frame이 존재하지 않을때입니다. 
    {
        Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if(mbMapUpdated && mpLastKeyFrame) //map update가 진행됬고, lastkeyframe이 존재할때 실행됩니다. 
    {
        const cv::Mat twb1 = mpLastKeyFrame->GetImuPosition(); //transformation 부분 imu데이터를 가져옵니다. 
        const cv::Mat Rwb1 = mpLastKeyFrame->GetImuRotation(); //rotation 부분 imu데이터를 가져옵니다. 
        const cv::Mat Vwb1 = mpLastKeyFrame->GetVelocity(); //velocity 값을 가져옵니다. 

        const cv::Mat Gz = (cv::Mat_<float>(3,1) << 0,0,-IMU::GRAVITY_VALUE); //IMU data의 gravity 값을 3,1 matrix로 가져옵니다. 
        const float t12 = mpImuPreintegratedFromLastKF->dT; //1번과 2번의 time 차이값입니다. 

        //위의 변수들을 통해서 실제 연산에 이용될 값들을 아래 변수들로 정의합니다. 

        cv::Mat Rwb2 = IMU::NormalizeRotation(Rwb1*mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias())); //bias값을 고려하여 delta rotation값을 가져오고 해당 값을 SVD(특이값분해) 작업을 진행합니다. 
        cv::Mat twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias()); //운동방정식 및 노이즈를 계산하여 최종 twb를 구합니다. 
        cv::Mat Vwb2 = Vwb1 + t12*Gz + Rwb1*mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias()); //velocity 부분도 동일합니다. 
        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2); //pose와 velocity를 predict할수 있는 변수들을 입력합니다. 
        mCurrentFrame.mPredRwb = Rwb2.clone(); //rwb2 matrix 저장
        mCurrentFrame.mPredtwb = twb2.clone(); //twb2 matrix 저장 
        mCurrentFrame.mPredVwb = Vwb2.clone(); //Vwb2 matrix 저장
        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias(); //bias 값 저장
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias; //current bias 값 저장
        return true;
    }
    else if(!mbMapUpdated) //만약에 map update가 진행되지 않았을때 실행됩니다.
    {
        const cv::Mat twb1 = mLastFrame.GetImuPosition(); //Keyframe이 아닌 일반 frame에서 data를 가져옵니다. 
        const cv::Mat Rwb1 = mLastFrame.GetImuRotation(); // 마찬가지입니다. 
        const cv::Mat Vwb1 = mLastFrame.mVw;
        const cv::Mat Gz = (cv::Mat_<float>(3,1) << 0,0,-IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        cv::Mat Rwb2 = IMU::NormalizeRotation(Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias)); //위에서 keyframe데이터로 했던 것과 동일합니다. 
        cv::Mat twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        cv::Mat Vwb2 = Vwb1 + t12*Gz + Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);
        mCurrentFrame.mPredRwb = Rwb2.clone();
        mCurrentFrame.mPredtwb = twb2.clone();
        mCurrentFrame.mPredVwb = Vwb2.clone();
        mCurrentFrame.mImuBias =mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}


void Tracking::ComputeGyroBias(const vector<Frame*> &vpFs, float &bwx,  float &bwy, float &bwz)
{
    const int N = vpFs.size(); //vpFs는 index값으로 추측됩니다. 즉 데이터 갯수를 뜻합니다. 
    vector<float> vbx; //gyro의 bias x 값입니다. 
    vbx.reserve(N); //memory 증가합니다. 
    vector<float> vby; //gyro의 bias y 값입니다.
    vby.reserve(N); //memory값을 증가합니다. 
    vector<float> vbz; //동일
    vbz.reserve(N); //동일

    cv::Mat H = cv::Mat::zeros(3,3,CV_32F); //32-bit floating-point number , 3x3 matrix 생성합니다. 
    cv::Mat grad  = cv::Mat::zeros(3,1,CV_32F); //32-bit floating-point number , 3x1 matrix 생성합니다. 
    for(int i=1;i<N;i++)  //bias의 최종 갯수만큼 연산을 시작합니다.
    {
        Frame* pF2 = vpFs[i]; //대상하는 bias의 앞 bias입니다.
        Frame* pF1 = vpFs[i-1]; //현재 대상이 되는 bias값입니다. 
        cv::Mat VisionR = pF1->GetImuRotation().t() * pF2->GetImuRotation(); //각 두개의 frame의 imu값을 곱합니다. (최종 rotation값, GT값으로 볼수있습니다.)
        cv::Mat JRg = pF2->mpImuPreintegratedFrame->JRg; //bias compute되기 전 original bias값입니다. 
        cv::Mat E = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaRotation().t() * VisionR; //delta rotation값과 vision R값을 곱해서 변화된 vision R값을 구합니다. 
        cv::Mat e = IMU::LogSO3(E); //logSO3로 다시 변환합니다. 
        assert(fabs(pF2->mTimeStamp - pF1->mTimeStamp - pF2->mpImuPreintegratedFrame->dT) < 0.01); //이건 오류메세지를 뽑기위한 라인입니다. 디버깅 작업이라고 할수 있습니다. 

        cv::Mat J = -IMU::InverseRightJacobianSO3(e) * E.t() * JRg; //마지막으로 e matrix와 transpose값과 JRg값을 연산해서 최종 bias matrix 값을 구합니다. 
        grad += J.t() * e; //기존e에 J.t를 연산하여 zero matrix에 축척합니다.
        H += J.t() * J; //기존 J에도 J.t를 연산하여 zero matrix에 축척합니다. 
    }

    cv::Mat bg = -H.inv(cv::DECOMP_SVD)*grad; //H의 pseudo-inverse matrix와 grap matrix를 연산하여 bg에 저장합니다. 
    bwx = bg.at<float>(0);
    bwy = bg.at<float>(1);
    bwz = bg.at<float>(2);

    for(int i=1;i<N;i++)
    {
        Frame* pF = vpFs[i];
        pF->mImuBias.bwx = bwx;
        pF->mImuBias.bwy = bwy;
        pF->mImuBias.bwz = bwz;
        pF->mpImuPreintegratedFrame->SetNewBias(pF->mImuBias);
        pF->mpImuPreintegratedFrame->Reintegrate();
    }
}

void Tracking::ComputeVelocitiesAccBias(const vector<Frame*> &vpFs, float &bax,  float &bay, float &baz)
{
    const int N = vpFs.size();
    const int nVar = 3*N +3; // 3 velocities/frame + acc bias
    const int nEqs = 6*(N-1);

    cv::Mat J(nEqs,nVar,CV_32F,cv::Scalar(0));
    cv::Mat e(nEqs,1,CV_32F,cv::Scalar(0));
    cv::Mat g = (cv::Mat_<float>(3,1) << 0,0, -IMU::GRAVITY_VALUE);

    for(int i=0;i<N-1;i++)
    {
        Frame* pF2 = vpFs[i+1];
        Frame* pF1 = vpFs[i];
        cv::Mat twb1 = pF1->GetImuPosition();
        cv::Mat twb2 = pF2->GetImuPosition();
        cv::Mat Rwb1 = pF1->GetImuRotation();
        cv::Mat dP12 = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaPosition(); 
        cv::Mat dV12 = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaVelocity();
        cv::Mat JP12 = pF2->mpImuPreintegratedFrame->JPa;
        cv::Mat JV12 = pF2->mpImuPreintegratedFrame->JVa;
        float t12 = pF2->mpImuPreintegratedFrame->dT;
        // Position p2=p1+v1*t+0.5*g*t^2+R1*dP12
        J.rowRange(6*i,6*i+3).colRange(3*i,3*i+3) += cv::Mat::eye(3,3,CV_32F)*t12;
        J.rowRange(6*i,6*i+3).colRange(3*N,3*N+3) += Rwb1*JP12;
        e.rowRange(6*i,6*i+3) = twb2-twb1-0.5f*g*t12*t12-Rwb1*dP12;
        // Velocity v2=v1+g*t+R1*dV12
        J.rowRange(6*i+3,6*i+6).colRange(3*i,3*i+3) += -cv::Mat::eye(3,3,CV_32F);
        J.rowRange(6*i+3,6*i+6).colRange(3*(i+1),3*(i+1)+3) += cv::Mat::eye(3,3,CV_32F);
        J.rowRange(6*i+3,6*i+6).colRange(3*N,3*N+3) -= Rwb1*JV12;
        e.rowRange(6*i+3,6*i+6) = g*t12+Rwb1*dV12;
    }

    cv::Mat H = J.t()*J;
    cv::Mat B = J.t()*e;
    cv::Mat x(nVar,1,CV_32F);
    cv::solve(H,B,x);

    bax = x.at<float>(3*N);    //N번째 bias를 가져옵니다. 
    bay = x.at<float>(3*N+1);
    baz = x.at<float>(3*N+2);

    for(int i=0;i<N;i++)
    {
        Frame* pF = vpFs[i]; //각 index의 frame data
        x.rowRange(3*i,3*i+3).copyTo(pF->mVw); //방정식의 해 x --> position 값과 velocity값이 들어가있습니다. 각 3~6 *i번째의 linear velocity값을 가져옵니다. 
        if(i>0) //i가 초기값이 아니라면 실행됩니다. 
        {
            pF->mImuBias.bax = bax; //bias업데이트 합니다.
            pF->mImuBias.bay = bay; 
            pF->mImuBias.baz = baz; 
            pF->mpImuPreintegratedFrame->SetNewBias(pF->mImuBias); //업데이트한 bias값을 갱신합니다. 
        }
    }
}


void Tracking::Track()
{

    if (bStepByStep)
    {
        while(!mbStep)
            usleep(500);
        mbStep = false;
    }

    if(mpLocalMapper->mbBadImu)
    {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        return;
    }

    Map* pCurrentMap = mpAtlas->GetCurrentMap();

    if(mState!=NO_IMAGES_YET)
    {
        if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
        {
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        }
        else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
        {
            cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
            if(mpAtlas->isInertial())
            {

                if(mpAtlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                    if(!pCurrentMap->GetIniertialBA2())
                    {
                        mpSystem->ResetActiveMap();
                    }
                    else
                    {
                        CreateMapInAtlas();
                    }
                }
                else
                {
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                    mpSystem->ResetActiveMap();
                }
            }

            return;
        }
    }


    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && !mbCreatedMap)
    {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPreIMU = std::chrono::steady_clock::now();
#endif
        PreintegrateIMU();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPreIMU = std::chrono::steady_clock::now();

        double timePreImu = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreIMU - time_StartPreIMU).count();
        vdIMUInteg_ms.push_back(timePreImu);
#endif

    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex>nMapChangeIndex)
    {
        // cout << "Map update detected" << endl;
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }


    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO)
            StereoInitialization();
        else
        {
            MonocularInitialization();
        }

        mpFrameDrawer->Update(this);

        if(mState!=OK) // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        if(mpAtlas->GetAllMaps().size() == 1)
        {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPosePred = std::chrono::steady_clock::now();
#endif

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if(mState==OK)
            {

                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if((mVelocity.empty() && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    //Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    //Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }


                if (!bOK)
                {
                    if ( mCurrentFrame.mnId<=(mnLastRelocFrameId+mnFramesToResetIMU) &&
                         (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO))
                    {
                        mState = LOST;
                    }
                    else if(pCurrentMap->KeyFramesInMap()>10)
                    {
                        cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        mState = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                        //mCurrentFrame.SetPose(mLastFrame.mTcw);
                    }
                    else
                    {
                        mState = LOST;
                    }
                }
            }
            else
            {

                if (mState == RECENTLY_LOST)
                {
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))
                    {
                        if(pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;

                        if (mCurrentFrame.mTimeStamp-mTimeStampLost>time_recently_lost)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                    else
                    {
                        // TODO fix relocalization
                        bOK = Relocalization();
                        if(!bOK && mCurrentFrame.mTimeStamp-mTimeStampLost>time_recently_lost_visual)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                }
                else if (mState == LOST)
                {

                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->KeyFramesInMap()<10)
                    {
                        mpSystem->ResetActiveMap();
                        cout << "Reseting current map..." << endl;
                    }else
                        CreateMapInAtlas();

                    if(mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }

        }
        else
        {
            // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
            if(mState==LOST)
            {
                if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPosePred = std::chrono::steady_clock::now();

        double timePosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPosePred - time_StartPosePred).count();
        vdPosePred_ms.push_back(timePosePred);
#endif


#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartLMTrack = std::chrono::steady_clock::now();
#endif
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
                bOK = TrackLocalMap();

            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else if (mState == OK)
        {
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            {
                Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                {
                    cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                    mpSystem->ResetActiveMap();
                }

                mState = RECENTLY_LOST;
            }
            else
                mState = RECENTLY_LOST; // visual to lost

            if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {
                mTimeStampLost = mCurrentFrame.mTimeStamp;
            }
        }

        // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        if((mCurrentFrame.mnId<(mnLastRelocFrameId+mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) && ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && pCurrentMap->isImuInitialized())
        {
            // TODO check this situation
            Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }

        if(pCurrentMap->isImuInitialized())
        {
            if(bOK)
            {
                if(mCurrentFrame.mnId==(mnLastRelocFrameId+mnFramesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                }
                else if(mCurrentFrame.mnId>(mnLastRelocFrameId+30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndLMTrack = std::chrono::steady_clock::now();

        double timeLMTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLMTrack - time_StartLMTrack).count();
        vdLMTrack_ms.push_back(timeLMTrack);
#endif

        // Update drawer
        mpFrameDrawer->Update(this);
        if(!mCurrentFrame.mTcw.empty())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        if(bOK || mState==RECENTLY_LOST)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty() && !mCurrentFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartNewKF = std::chrono::steady_clock::now();
#endif
            bool bNeedKF = NeedNewKeyFrame();




            // Check if we need to insert a new keyframe
            if(bNeedKF && (bOK|| (mState==RECENTLY_LOST && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))))
                CreateNewKeyFrame();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();

            double timeNewKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndNewKF - time_StartNewKF).count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(pCurrentMap->KeyFramesInMap()<=5)
            {
                mpSystem->ResetActiveMap();
                return;
            }
            if ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO))
                if (!pCurrentMap->isImuInitialized())
                {
                    Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    return;
                }

            CreateMapInAtlas();
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }




    if(mState==OK || mState==RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if(!mCurrentFrame.mTcw.empty())
        {
            cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState==LOST);
        }

    }
}

void Tracking::StereoInitialization()
{
    // N: Keypoints의 개수
    if(mCurrentFrame.N>500) // Keypoints의 개수가 500개 이상일 때 초기화를 진행하고, / 이하인 경우에는 초기화를 진행하지 않음
    {
        if (mSensor == System::IMU_STEREO) // Stereo-Inertial mode인 경우
        {
            // CurrentFrame과 LastFrame에 대한 IMU-Preintegration 정보가 존재하는지 체크
            if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
            {
                cout << "not IMU meas" << endl;
                return; // CurrentFrame과 LastFrame에 대한 IMU-Preintegration 정보가 존재하지 않으면 초기화 종료
            }

            // CurrentFrame과 LastFrame에 대한 IMU-Preintegration의 평균 가속도값의 차이가 0.5 이상인지 확인
            if (cv::norm(mCurrentFrame.mpImuPreintegratedFrame->avgA-mLastFrame.mpImuPreintegratedFrame->avgA)<0.5)
            {
                cout << "not enough acceleration" << endl;
                return; // CurrentFrame과 LastFrame에 대한 IMU-Preintegration의 평균값의 차이가 0.5 미만인 경우 초기화 종료(충분한 움직임 값을 획득해야됨)
            }
            
            // Last KeyFrame에 대한 IMU-Preintegration 객체를 초기화 (메모리를 해제)
            if(mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            // IMU::Bias(): IMU sensor의 noise를 반환 [acceleration(3), angular velocity(3)]
            //            : -> bax,bay,baz,bwx,bwy,bwz
            // *mpImuCalib: IMU-Camera에 대한 Transformation matrix 정보 (Calibration 정보)
            //            : cv::Mat Tbc;
            //            : cv::Mat Cov, CovWalk;
            // IMU::Preintegrated(): 딱히 계산을 하지는 않음. White noise와 Random walk noise에 대한 coveriance matrix와 noise값을 복사하여 초기화만함
            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);

            // mpImuPreintegratedFromLastKF 객체의 주소값을 넘김
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        // CurrentFrame을 Global-coordinate으로 설정함: Origin(0,0,0)
        if (mSensor == System::IMU_STEREO)
        {
            cv::Mat Rwb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0,3).colRange(0,3).clone(); // [Rotation]    imu -> cam
            cv::Mat twb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0,3).col(3).clone();        // [Translation] imu -> cam
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, cv::Mat::zeros(3,1,CV_32F));       // world -> cam -> body
        }
        else
            mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        // KeyFrame에 대한 객체를 생성합니다.
        //   - mCurrentFrame로부터 CurrentFrame의 Left_image, Right_image를 저장합니다.
        //   - mCurrentFrame로부터 Global pose에 대한 정보(Tcw)와 IMU bias정보를 저장합니다.
        //   - mpAtlas->GetCurrentMap()로부터 현재 map에 대한 id정보를 저장합니다.
        //   - mpKeyFrameDB의 정보를 그대로 저장합니다. (정확한 용도는 잘몰르겠음 ???)
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

        // Insert KeyFrame in the map
        // 생성된 KeyFrame 객체를 추가합니다.
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        if(!mpCamera2){ // FishEye가 아니면
            // MapPoint를 생성합니다.
            // mCurrentFrame.N은 keypoint의 개수
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                // Feature_point에 대한 Depth 정보를 가져옵니다.
                float z = mCurrentFrame.mvDepth[i];                
                if(z>0) // depth값이 존재하면
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i); // Camera perspective model을 이용하여 3D map-point를 계산합니다.
                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpAtlas->GetCurrentMap()); // Global(world) Map포인트로 변환후 저장합니다.
                    pNewMP->AddObservation(pKFini,i); // Current Frame에서 관찰된 keypoint를 추가합니다.
                    pKFini->AddMapPoint(pNewMP,i);    // KeyFrame 객체에 3D Map-point를 추가합니다.
                    pNewMP->ComputeDistinctiveDescriptors(); // 현재 keypoint에 대한 Descriptor를 계산한다. (정확히 잘 몰르겠음)
                                                             // Map-point가 추가적으로 생겼을때, 추가할지 기존의 map-point에 업데이트할 지, 확인하기 위한 Descriptor를 생성하는것 같음
                                                             // 또한 BA와 tracking features의 수에 대하여 더 좋은 descriptor가 생성된다면, descriptor를 갱신하는 것 같음
                    pNewMP->UpdateNormalAndDepth(); // MapPoint에 대하여 Normal vector와 Depth 정보를 업데이트
                    mpAtlas->AddMapPoint(pNewMP);   // Atlas객체에 3D Map-point를 추가합니다.

                    mCurrentFrame.mvpMapPoints[i]=pNewMP; // 갱신된 Map-point정보를 업데이트해줌
                }
            }
        } else{ // fisheye에 대한 내용(내용은 위와 유사함)
            for(int i = 0; i < mCurrentFrame.Nleft; i++){
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if(rightIndex != -1){
                    cv::Mat x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini,i); 
                    pNewMP->AddObservation(pKFini,rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP,i);
                    pKFini->AddMapPoint(pNewMP,rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft]=pNewMP;
                }
            }
        }

        // 출력문
        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

        mpLocalMapper->InsertKeyFrame(pKFini); // LocalMapper에 current key-frame에 대한 정보를 추가

        mLastFrame = Frame(mCurrentFrame);      // currentframe에 대한 정보를 last-keyframe에 정보를 저장하고, 새로운 frame을 받을 준비
        mnLastKeyFrameId=mCurrentFrame.mnId;    // currentframe에 대한 id정보를 저장
        mpLastKeyFrame = pKFini;                // current key-frame에 대한 정보를 last-keyframe에 저장
        mnLastRelocFrameId = mCurrentFrame.mnId;// current key-frame에 대한 id 정보를 last-keyframe에 저장

        mvpLocalKeyFrames.push_back(pKFini);    // local keyframe에 저장
        mvpLocalMapPoints=mpAtlas->GetAllMapPoints(); // Atlas의 map-point의 vector 포인터를 lcoal-map-point에 넘겨줌
        mpReferenceKF = pKFini; // 현재 KF를 reference KF로 설정
        mCurrentFrame.mpReferenceKF = pKFini; // 현재 KF를 currentFrame의 referenceKF로 설정

        // mvpLocalMapPoints를 Atlas의 refrence KF로 설정
        // Atlas의 ReferenceMapPoint는 Map을 Drawing할 때 사용됨
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        // KF를 저장
        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        // Map에 point를 drawing할때 필요한 좌표계 변환 정보를 저장
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}


void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {

            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            if (mSensor == System::IMU_MONOCULAR)
            {
                if(mpImuPreintegratedFromLastKF)
                {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            }
            return;
        }
    }
    else
    {
        if (((int)mCurrentFrame.mvKeys.size()<=100)||((mSensor == System::IMU_MONOCULAR)&&(mLastFrame.mTimeStamp-mInitialFrame.mTimeStamp>1.0)))
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Rcw,tcw,mvIniP3D,vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();

        }
    }
}



void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpAtlas->AddMapPoint(pMP);
    }


    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

    pKFcur->PrintPointDistribution();

    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if(mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f/medianDepth; // 4.0f
    else
        invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50) // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_NORMAL);
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
    }


    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

    cv::Mat deltaT = vKFs.back()->GetPose()*vKFs.front()->GetPoseInverse();
    mVelocity = cv::Mat();
    Eigen::Vector3d phi = LogSO3(Converter::toMatrix3d(deltaT.rowRange(0,3).colRange(0,3)));


    double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;

    initID = pKFcur->mnId;
}


// #############################################################################################################################################
// 정진용 연구원님의 블로그에서 ORBSLAM-Atlas 내용 인용 (http://jinyongjeong.github.io/2019/11/07/IROS2019_SLAM_list/)
// [IROS 2019] ORBSLAM-Atlas: a robust and accurate multi-map system
// ORBSLAM-Atals란?
// - 기존 ORB-SLAM은 tracking loss가 발생하면 mapping이 중단되지만, ORB-SLAM-Atlas는 tracking loss가 발생하면 새로운 submap을 생성하고, 기존의 submap과 동일 영역을 찾으면 merge
// - 제한없이 여러개의 submap을 다루기 위한 시스템
// - submap간의 overlap을 검출 및 merge
// - 각 atlas는 효율적인 Place recognition을 위해 각각의 unique한 DBow를 갖고 있음
// #############################################################################################################################################
void Tracking::CreateMapInAtlas()
{
    mnLastInitFrameId = mCurrentFrame.mnId; // currentFrame의 ID를 LastFrame ID에 복사
    
    // 새로운 map을 생성
    // map들은 vector<Map> 자료형에 의해 관리됨 (Map.h에서의 class 객체)
    // 기존 맵을 있다면 저장하고, 새로운 맵을 생성
    mpAtlas->CreateNewMap(); 

    // Stereo+IMU or Mono+IMU 시스템인 경우
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
        mpAtlas->SetInertialSensor(); // IMU가 사용되는지 유무를 설정하는 함수 // mbIsInertial = true가 전부
    mbSetInit=false; // 초기화 유무에 대한 flag

    mnInitialFrameId = mCurrentFrame.mnId+1; // Initial-Frame id를 마지막 프레임의 'ID+1'로 할당
                                             // 새로운 atlas-map에 대한 시작 프레임 id를 설정
    mState = NO_IMAGES_YET; // 새로운 이미지가 없는 상태로 설정
    
    // 새로 생성된 atlas-map에 대한 변수들은 마지막 KF에 대한 변수들의 값을 가져오고, 재시작
    mVelocity = cv::Mat();
    mnLastRelocFrameId = mnLastInitFrameId; // 기존 map에 대한 마지막 frame id(mnLastInitFrameId)는 relocalization을 위한 시작 프레임으로 설정
    Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId+1), Verbose::VERBOSITY_NORMAL);
    mbVO = false; // 마지막 KF에 충분한 MapPoint의 존재 유무에 대한 flag

    // mono || mono-imu인 경우
    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        // 초기화 변수에 대한 제거
        if(mpInitializer) 
            delete mpInitializer;
        
        // 초기화 변수에 대한 재할당
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    // mono-imu || stereo-imu인 경우
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ) && mpImuPreintegratedFromLastKF)
    {
        // 마지막 KF에 대한 preintegration 변수 메모리를 제거
        delete mpImuPreintegratedFromLastKF;

        // 새로운 preintegration객체를 생성
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
    }

    // mpLastKeyFrame변수 초기화
    if(mpLastKeyFrame)
        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

    // mpReferenceKF 초기화
    if(mpReferenceKF)
        mpReferenceKF = static_cast<KeyFrame*>(NULL);

    // 각 변수 초기화
    mLastFrame = Frame();    // mLastFrame 초기화
    mCurrentFrame = Frame(); // mCurrentFrame 초기화
    mvIniMatches.clear();    // mvIniMatches 초기화

    mbCreatedMap = true;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
    {
        cout << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    //mCurrentFrame.PrintPointDistribution();


    // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    // TODO check these conditions
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        return true;
    else
        return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
        {
            break;
        }
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);   // matcher라는 ORBmatcher 클래스 변수 선언

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    // 만약, Localization Mode일 경우에는 "visual odometry"용 points를 만든다.
    UpdateLastFrame();  // Reference keyframe에 따라 마지막 frame의 camera pose를 업데이트

    // 만약 current map에 IMU 초기화가 되어있고, 
    // 현재 Frame의 ID가 마지막 Relocalization 할 때 사용했던 Frame ID + IMU를 초기화하는 Frame ID보다 클 경우
    // (IMU data는 지속적으로 들어오는데, 이미지에서 tracking이 제대로 되고 있지 않다고 판단할 때)
    if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
    {
        // Predict state with IMU if it is initialized and it doesnt need reset
        PredictStateIMU();  // IMU의 state만 추정 , pose estimation을 진행 (IMU vs ORB Matching)일때 IMU를 활용하여 state를 추정
        return true;    // Motion model을 가정한 것이 true 
    }
    else   // Frame 간격이 이미지 tracking이 잘 되고 촘촘하게 되어있을 때 
    {
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);   // Current Frame의 카메라 Pose를 새로 지정
        // Pose를 구할 때 등속도 운동을 가정하여 World to Camera Translation Matrix에 mVelocity를 곱해준다.
        // mVelocity는 IMU data를 이용하여 계산을 진행한 것 (지속적으로 계산을 진행)
    }

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
    // Current Frame의 MapPoint vector를 초기화

    // Project points seen in previous frame
    int th;

    if(mSensor==System::STEREO) // Threshold 값 지정, ORB Matcher클래스 안에 있는 함수를 사용하는데 Window Size 크기를 정해줌
        th=7;  
    else
        th=15;  

    // Current Frame과 Last Frame의 Match하는 point 갯수 찾기
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

    // If few matches, uses a wider window search
    // Match 되는 point의 갯수가 적을 경우 다시 Window의 Size를 키워 Match하는 point의 갯수를 찾는다.
    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        // Current Frame의 MapPoint vector를 초기화

        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
        Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);

    }

    // Window의 Size를 키웠는데도 matching되는 point의 갯수가 부족한 경우
    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            return true;    // IMU_MONOCULAR, IMU_STEREO 인 경우는 Motion model을 가정한 것이 true
        else
            return false;   // MONOCULAR, STEREO인 경우는 Motion model을 가정한 것이 false
    }


    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);  // Update된 matching point를 이용하여 Current Frame의 pose를 최적화

    // Discard outliers - Outlier 버리기
    int nmatchesMap = 0;
    // Current Frame의 Key points만큼 for문
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL); // Current Frame MapPoint 초기화
                mCurrentFrame.mvbOutlier[i]=false;  // Current Frame MapPoint Outlier 상태 초기화

                // 왼쪽이나 오른쪽 TrackInView 변수를 false로 지정
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false; // 현재 Current Frame의 Map point가 보인다는 변수(TrackInView)를 false로 선언
                    // Frame Class에 isInFrustum()함수에서만 true로 만들어준다.
                }
                else{
                    pMP->mbTrackInViewR = false;  // 현재 Current Frame의 Map point가 보인다는 변수(TrackInView)를 false로 선언
                }

                pMP->mnLastFrameSeen = mCurrentFrame.mnId;  // Map point에 있는 Last Frame 보여지는 변수를 Current Frame으로 변경
                nmatches--; // Outlier 제거
            }

            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)    
                nmatchesMap++;  // Matching되는 Map point 갯수 증가
        }
    }

    if(mbOnlyTracking)  // 만약 local mapping이 비활성화 되어있고 Localizaation만 수행하는 경우
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;    // Match되는 point의 갯수에 따라 Motion model인지 아닌지 결정 
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        return true;    // IMU_MONOCULAR, IMU_STEREO 인 경우는 Motion model을 가정한 것이 true
    else
        return nmatchesMap>=10; // Outlier를 제거한 Map point의 개수에 따라 Motion model인지 아닌지 결정
}

bool Tracking::TrackLocalMap()
{

    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    mTrackedFr++;   // 디버깅용으로 넣어둔 변수 (Tracking에 다른 곳에서 안씀)

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartLMUpdate = std::chrono::steady_clock::now();
#endif
    UpdateLocalMap();   // Local Map을 Update
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartSearchLP = std::chrono::steady_clock::now();

    double timeUpdatedLM_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_StartSearchLP - time_StartLMUpdate).count();
    vdUpdatedLM_ms.push_back(timeUpdatedLM_ms);
#endif

    SearchLocalPoints();    // Local Map Point를 검색 (Update하는 의미로 이해)
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartPoseOpt = std::chrono::steady_clock::now();

    double timeSearchLP_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_StartPoseOpt - time_StartSearchLP).count();
    vdSearchLP_ms.push_back(timeSearchLP_ms);
#endif

    // TOO check outliers before PO
    // Pose Optimization을 하기 전에 Outlier check
    // Tracking 에서는 aux1, aux2 변수가 쓰이지 않는 것으로 보아 디버깅용으로 추정
    int aux1 = 0, aux2=0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    int inliers;
    if (!mpAtlas->isImuInitialized())   // Atlas에 IMU가 초기화 되어 있지 않다면
        Optimizer::PoseOptimization(&mCurrentFrame);    // Current Frame을 이용하여 camera pose를 최적화
    else    // Atlas에 IMU가 초기화 되어 있다면
    {
        // Current Frame의 ID보다 Last Relocalization Frame ID와 IMU를 Reset한 ID의 합이 크거나 같다면
        if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
        {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);     // Current Frame을 이용하여 camera pose를 최적화
        }
        else    // Current Frame의 ID보다 Last Relocalization Frame ID와 IMU를 Reset한 ID의 합이 작다면
        {
            if(!mbMapUpdated)   // Map update가 되지 않았을 때
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                // Last Frame의 이용하여 Camera-Inertial pose를 최적화 (?) - Last Frame을 활용
            }
            else    // Map update가 되었을 때
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
                // Last Key Frame의 이용하여 Camera-Inertial pose를 최적화 (?) - Last KeyFrame을 활용
            }
        }
    }
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndPoseOpt = std::chrono::steady_clock::now();

    double timePoseOpt_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPoseOpt - time_StartPoseOpt).count();
    vdPoseOpt_ms.push_back(timePoseOpt_ms);
#endif

    vnKeyFramesLM.push_back(mvpLocalKeyFrames.size());  // Local Key Frame의 size를 vnKeyFramesLM에 저장
    vnMapPointsLM.push_back(mvpLocalMapPoints.size());  // Local Map Point의 size를 vnMapPointsLM에 저장

    // Tracking 에서는 aux1, aux2 변수가 쓰이지 않는 것으로 보아 디버깅용으로 추정
    aux1 = 0, aux2 = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    mnMatchesInliers = 0;   // Match 된 Inliers counter를 위한 변수

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)    // Current Frame의 Key points를 for문
    {
        if(mCurrentFrame.mvpMapPoints[i])   // Map point 하나씩 돌기 위한 for문
        {
            if(!mCurrentFrame.mvbOutlier[i])    // Outlier가 아니라면
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound(); // Map point가 found 되었다는 변수를 증가
                if(!mbOnlyTracking) // Mapping을 진행하고 있다면 (Localization mode가 아니라는 뜻)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0) // Current Frame에서 Map point가 관찰이 될 경우
                        mnMatchesInliers++; // Match 된 Inlier 변수 증가
                }
                else    // Localization mode일 때
                    mnMatchesInliers++; // Match 된 Inlier 변수 증가
            }
            else if(mSensor==System::STEREO)    // Outlier인데 Sensor가 STEREO일 경우
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);   // Map point를 NULL 값으로 초기화
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    // Inlier의 갯수를 통해 Tracking이 되고 있는지 안되고 있는지 판단
    mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    // Current Frame의 ID보다 Last Relocalization Frame ID와 Max Frame(FPS)의 합이 더 클 경우,
    // Match 된 inlier의 갯수가 50 이하일 경우
        return false;   // Local Map을 Tracking하지 못한다고 판단

    if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
    // Match 된 Inlier의 수가 10을 넘고, Tracking state가 RECENTLY_LOST일 경우
        return true;    // Local Map을 Tracking을 하고 있다고 판단


    if (mSensor == System::IMU_MONOCULAR)   // Mode가 Mono-Inertial일 때
    {
        if(mnMatchesInliers<15) // Match 된 inlier의 갯수가 15 이하일 경우
        {
            return false;   // Local Map을 Tracking하지 못한다고 판단
        }
        else    
            return true;    // Local Map을 Tracking을 하고 있다고 판단
    }

    else if (mSensor == System::IMU_STEREO) // Mode가 Stereo-Inertial일 때
    {
        if(mnMatchesInliers<15) // Match 된 inlier의 갯수가 15 이하일 경우
        {
            return false;   // Local Map을 Tracking하지 못한다고 판단
        }
        else
            return true;    // Local Map을 Tracking을 하고 있다고 판단
    }

    else    // Mode가 Stereo 그리고 Monocular일 경우
    {
        if(mnMatchesInliers<30) // Match 된 inlier의 갯수가 30 이하일 경우
            return false;   // Local Map을 Tracking하지 못한다고 판단
        else
            return true;    // Local Map을 Tracking을 하고 있다고 판단
    }
}

bool Tracking::NeedNewKeyFrame()
{
    if(((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if (mSensor == System::IMU_STEREO && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }

    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    {
        return false;
    }

    // Return false if IMU is initialazing
    if (mpLocalMapper->IsInitializing())
        return false;
    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;

    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;

            }
        }
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2) thRefRatio = 0.75f;

    if(mSensor==System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) && mnMatchesInliers>15);

    // Temporal condition for Inertial cases
    bool c3 = false;
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR))) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4=true;
    else
        c4=false;

    if(((c1a||c1b||c1c) && c2)||c3 ||c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(mpLocalMapper->IsInitializing())
        return;

    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mpAtlas->isImuInitialized())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mpLastKeyFrame)
    {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    }
    else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
    {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
    }

    if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
    {
        mCurrentFrame.UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if(mSensor == System::IMU_STEREO)
            maxPoint = 100;

        vector<pair<float,int> > vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D;

                    if(mCurrentFrame.Nleft == -1){
                        x3D = mCurrentFrame.UnprojectStereo(i);
                    }
                    else{
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF,i);

                    //Check if it is a stereo observation in order to not
                    //duplicate mappoints
                    if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
                        pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
                {
                    break;
                }
            }

            Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);

        }
    }


    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
    //cout  << "end creating new KF" << endl;
}

/* !
* @brief  : Local Map Points와 Current Frame의 Map Points를 매칭시켜주는 함수
* @param  : None
* @return : None
*/
void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    //^ Current frame에서 이미 매칭된 MapPoint들에 대해서, 즉 CurrentFrame의 KeyPoints
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            //^ Bad MapPoint -> NULL로
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                //^ For visualization
                //^ 나중에 GetFoundRatio에서 사용됨. 
                //^ FoundRatio = Found/Visible, 일정 FoundRatio 밑으로 떨어진 MapPoint는 삭제됨.  
                pMP->IncreaseVisible();
                //^ 해당 MapPoint가 마지막으로 보여진 Frame은 CurrentFrame이다.
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                //^ TrackInView에서 제외
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    //^ 이전에 찾은 LocalMapPoints 중에서
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        //^ 이미 Matching 된 LocalMapPoint는 skip한다.
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        //^ Bad LocalMapPoint는 skip한다.
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        //^ 그 외의 경우, 
        //^ LocalMapPoint가 CurrentFrame의 Frustum에 들어오면,
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            //^ Visible.
            pMP->IncreaseVisible();
            //^ Match됨.
            nToMatch++;
        }
        //^ For visualization
        //^ LocalMapPoint 중에서 CurrentFrame의 Frustnum에 들어온 MapPoint는 ProjectPoints에 저장하여 
        //^ 나중에 Drawer에서 시각화해준다.
        if(pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    //^ 하나라도 LocalMapPoints 중에서 CurrentFrame의 Frustum에 들어온게 있으면(=Has seen) 된게 있으면,
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;

        //^ 시스템 환경에 따라 ORB Matcher의 threshold 다름.
        if(mSensor==System::RGBD)
            th=3;
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=3;
        }
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15;

        //^ LocalMapPoints의 descriptor와 CurrentFrame의 KeyPoints의 descriptor 간 matching
        //^ Match된 KeyPoints 갯수 반환
        //^ 그러나 쓰이진 않음.
        //^ CurrentFrame에 Matching된 MapPoint를 저장하는 멤버인 mvpMapPoints에 Match된 KeyPoints 저장하는 함수
        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);  // Local Map points들을 활용하여 Reference Map point 지정

    // Local Key Frames + Local Map Points >>> Local Map
    UpdateLocalKeyFrames(); // Local KeyFrame update
    UpdateLocalPoints();    // Local Map points update
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();  // Map point들이 담겨있는 Vector 초기화

    int count_pts = 0;  // Map Point의 갯수를 count하기 위한 변수

    // Local Key Frame을 reverse로 for문 - 최신 Key Frame부터 check하기 위해서
    for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();  // Match된 Map point들을 이용하여 vpMPs 변수 초기화

        // 하나의 Key Frame에 있는 Map point들을 for문을 이용하여 하나씩 순회
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)  // Map point가 없다면
                continue;   // Skip
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)   // Current Frame이 Reference Frame의 ID와 같다면
                continue;   // Skip

            if(!pMP->isBad())   // Map point의 상태가 좋다면, Reference Frame과 Current Frame이 다르면
            {
                count_pts++;    // count 변수 증가
                mvpLocalMapPoints.push_back(pMP);   // Local Map Points 벡터에 추가
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;   // Current Frame을 Reference Frame으로 update
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter; // <keyFrame, 얼마나 Voting 했는지 count하기 위한 int형> 으로 map 자료구조를 활용하여 변수 선언

    // Atlas에 IMU가 초기화 되어 있지 않거나, 현재 Current Frame의 ID보다 마지막 Relocalization Frame의 ID +2가 더 클 경우
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
    {
        for(int i=0; i<mCurrentFrame.N; i++)    // Current Frame의 Key point를 for문으로 순회
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];  // Current Frame의 Map point 하나를 pointer로 선언
            if(pMP)
            {
                if(!pMP->isBad())   // Map point가 Good point라면
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    // Current Frame의 Map point를 관찰하고 있는 여러 Keyframe을 observation 변수를 이용하여 저장
                    // observations에는 현재 Map point를 보고 있는 Keyframe에 대한 index 정보들을 가지고 있다.  
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;   // Voting을 진행 - for문을 통해서 map point를 보고 있는 Key frame -> 가장 많이 겹쳐져 있는 key frame 선별
                }
                else    // Map point가 Bad point라면
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL; // NULL 값으로 초기화
                }
            }
        }
    }
    else    // Atlas에 IMU 초기화 되어 있거나, Current Frame의 ID보다 Relocalization Frame의 ID +2가 더 작을 경우
    {
        for(int i=0; i<mLastFrame.N; i++)   // Last Frame의 Key point를 for문으로 순회
        {
            // Using lastframe since current frame has not matches yet
            // Current Frame이 match가 일어나지 않았기 때문에 Last Frame을 사용합니다.
            if(mLastFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i]; // Last Frame의 Map point 하나를 pointer로 선언
                if(!pMP)    // Map point가 없다면
                    continue;   // Skip
                if(!pMP->isBad())   // Map point가 Good point라면
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    // Last Frame의 Map point를 관찰하고 있는 여러 Keyframe을 observation 변수를 이용하여 저장
                    // observations에는 현재 Map point를 보고 있는 Keyframe에 대한 index 정보들을 가지고 있다. 
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;   // Voting을 진행 - for문을 통해서 map point를 보고 있는 Key frame -> 가장 많이 겹쳐져 있는 key frame 선별
                }
                else    // Map point가 Bad point라면
                {
                    mLastFrame.mvpMapPoints[i]=NULL;    // NULL 값으로 초기화
                }
            }
        }
    }

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;  // Key Frame을 pointer 변수를 활용하여 지정

        if(pKF->isBad())    // Bad Key Frame일 경우 
            continue;   // Skip

        if(it->second>max)  // 최대로 Voting이 일어난 Key Frame 업데이트
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);   // Key Frame을 Local Key Frame Vector에 담는다.  
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId; // 
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    // Local Key Frame의 for문을 돌면서 인접한 포함되어 있지 않은 key frame을 포함시킨다.
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80) // Key Frame Vector의 Size가 80이 넘는다면
            break;  // for문을 빠져 나온다.

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);    // N은 size, 가장 좋은 Covisibility를 가지고 있는 keyframe

        // for문을 이용하여 좋은 Covisibility를 가지고 있는 KeyFrame 순회
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())  // 좋은 Key Frame이라면
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                // 인접한 Key frame의 Reference Frame의 ID를 현재 current frame의 ID와 같지 않다면
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);  // 인접한 Key frame도 Local key frame으로 집어넣기
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;  // 인접한 Key Frame의 Reference Frame을 Current Frame의 ID로 대입
                    break;  // 인접한 Key Frame 순회 break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();   // Covisibility Graph 중 자식 노드에 해당하는 Key Frame 집합을 가져온다.
        // for문을 이용하여 좋은 Covisibility를 가지고 있는 자식 노드 KeyFrame 순회
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())  // 좋은 Key Frame이라면
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                // 자식 노드 Key frame의 Reference Frame의 ID를 현재 current frame의 ID와 같지 않다면
                {
                    mvpLocalKeyFrames.push_back(pChildKF);  // 자식 노드 Key frame도 Local key frame으로 집어넣기
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;  // 자식 노드 Key Frame의 Reference Frame을 Current Frame의 ID로 대입
                    break;  // 자식 노드 Key Frame 순회 break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();    // Covisibility Graph 중 부모 노드에 해당하는 Key Frame 집합을 가져온다.
        if(pParent) // 부모 노드 Key Frame이 존재한다면
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            // 부모 노드 Key frame의 Reference Frame의 ID를 현재 current frame의 ID와 같지 않다면
            {
                mvpLocalKeyFrames.push_back(pParent);   // 부모 노드 Key frame도 Local key frame으로 집어넣기
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;   // 부모 노드 Key Frame의 Reference Frame을 Current Frame의 ID로 대입
                break;  // for문을 빠져 나온다.
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    // IMU모드일 때 임시적인 key frame을 update
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) &&mvpLocalKeyFrames.size()<80)
    {
        KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;  // Current Frame에서 Last Key Frame을 임시 Key Frame으로 저장

        const int Nd = 20;
        for(int i=0; i<Nd; i++){
            if (!tempKeyFrame)
                break;
            if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            // 임시 노드 Key frame의 Reference Frame의 ID를 현재 current frame의 ID와 같지 않다면
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);  // 임시 노드 Key frame도 Local key frame으로 집어넣기
                tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;  // 임시 노드 Key Frame의 Reference Frame을 Current Frame의 ID로 대입
                tempKeyFrame=tempKeyFrame->mPrevKF; // 순회를 위해 Temp Key Frame의 이전 Key Frame을 대입
            }
        }
    }

    if(pKFmax)  // voting을 통해 선별된 가장 좋은 Key frame
    {
        mpReferenceKF = pKFmax; // Reference Key Frame으로 대입
        mCurrentFrame.mpReferenceKF = mpReferenceKF;    // Current Frame의 Reference Key Frame으로 대입
    }
}


/* !
* @brief  : Tracking이 LOST 되었을 때, Relocalization 시켜주는 함수
* @param  : None
* @return : Boolean
*/
bool Tracking::Relocalization()
{
    Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    //^ CurrentFrame이 가지고 있는 ORB descriptor로부터,
    //^ BoW vector와 Feature vector를 계산하여 CurrentFrame의 멤버(mBowVec, mFeatVec)에 update한다.
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    //^ Atlas에 있는 CurrentMap으로 부터 CurrentFrame의 BoW를 공유하는 KeyFrame들 중에서,
    //^ similarity score가 75% 이상인 KeyFrame들을 Relocalization 후보 Keyframe으로 뽑는다. 
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

    //^ 후보 KeyFrame이 없으면 Relocalization fail.
    if(vpCandidateKFs.empty()) {
        Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<MLPnPsolver*> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    
    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    //^ 전체 후보 KeyFrame에 대해서,
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        //^ KeyFrame이 Bad면 discard
        //^ Bad의 의미 : 버려지는 KeyFrame, 어디선가 erase되어 메모리 해제를 기다리는 KeyFrame
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            //^ 후보 KeyFrame의 descriptor와 CurrentFrame의 descriptor 간 matching
            //^ vvpMapPointMatches에는 matching된 MapPoint가 담김.
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            
            //^ match된 MapPoint가 적으면 discard
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            //^ match된 MapPoint가 충분하면,
            else
            {
                //^ Solve PnP
                //^ Camera의 intrinsic parameter,
                //^ World coordinate 상의 3차원 Points,
                //^ Image(Frame) 상의 2차원 Points
                //^ 를 알고 있을 때 Camera pose를 예측하는 solver
                //^ MLPnP : Maximum Likelihood solution to the Perspective-n-Point
                MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,6,0.5,5.991);  //This solver needs at least 6 points
                //^ PnPsolver 객체를 현재 KeyFrame index에 저장
                vpMLPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }
    //^ Vector 상세 설명
    //^ 후보 KeyFrame (vpCandidateKFs)          : [a, b, c, d, e, f ...] 
    //^ Discarded KeyFrame (vbDiscarded)        : [1, 1, 0, 1, 0, 0 ...], 0: FALSE, 1: TRUE
    //^ MLPnPSovler 객체 (vpMLPnPsolvers)       : [-, -, S, -, S, S ...], -: NULL, S: 객체
    //^ Match된 MapPoint (vvpMapPointMatches)   : [-, -, M, -, M, M ... ], -: NULL, M: Match된 MapPoints vector(KF의 MapPoints)  

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    //^ 후보 MLPnPsolver가 없거나 Match될 때까지,
    while(nCandidates>0 && !bMatch)
    {
        //^ 후보 KeyFrame들에 대해서,
        for(int i=0; i<nKFs; i++)
        {
            //^ Discarded KeyFrame은 skip
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            //^ Match된 MapPoint들(vvpMapPointMatches)에 대해서
            //^ 5 iterate RANSAC 수행
            //^ Tcw : RANSAC을 통해 나온 Camera pose
            MLPnPsolver* pSolver = vpMLPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            //^ RANSAC Fail (1. over iteration, 2. MapPoints가 6개보다 작을 때)
            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            //^ RANSAC 성공
            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                //^ RANSAC 결과 Camera Pose를 CurrentFrame의 pose로 copy
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound; 

                const int np = vbInliers.size();

                //^ RANSAC inlier들에 대해서,
                for(int j=0; j<np; j++)
                {
                    //^ Inlier 이면,
                    if(vbInliers[j])
                    {
                        //^ 해당 index의 MapPoint는 CurrentFrame에서 match된 MapPoint.
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        //^ sFound(std::set) : Match된 MapPoint들 중에서 RANSAC의 inlier는 Found로 간주(중복 없음) 
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    //^ Inlier가 아니면,
                    else
                        //^ 해당 index의 MapPoint는 CurrentFrame에서 NULL
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }
                
                //^ Match된 MapPoint들로 graph를 이루고 graph optimization을 통해 camera pose를 구함.
                //^ nGood : nInitialCorrespondences-nBad
                //^ -> Initial vertex 갯수(nInitialCorrespondences)에서 optimization 결과 outlier(nBad)로 판명난 vertex 갯수를 뺀 값
                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                //^ nGood이 너무 적으면 다음 KeyFrame으로
                if(nGood<10)
                    continue;

                //^ Outlier로 판명난 KeyPoint들은 set NULL.
                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                //^ nGood이 아쉬우면 한번 더
                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    //^ ORB matcher의 parameter 조금더 느슨하게 해서 match되는 MapPoint 추가로 찾음
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    //^ 추가로 match된 MapPoint와 nGood이 50이 넘으면,
                    if(nadditional+nGood>=50)
                    {
                        //^ 한번 더 optimize
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        //^ 한번 더..
                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            //^ 이번엔 parameter 빡세게(이미 많은 points들이 optimized 되었기 때문)
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            //^ 진짜 마지막..
                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }

                //^ Good match
                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    //^ 다른 후보 KeyFrame 볼 것도 없이 break (이 정도면 relocalization 성공)
                    break;
                }
            }
        }
    }

    //^ Good match가 없음.
    if(!bMatch)
    {
        return false;
    }
    //^ Relocalization 성공!
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "Relocalized!!" << endl;
        return true;
    }
}

void Tracking::Reset(bool bLocMap)
{
    Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }


    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clear();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }
    mbSetInit=false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap)
{
    Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    Map* pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();

    mnLastInitFrameId = Frame::nNextId;
    mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    list<bool> lbLost;
    unsigned int index = mnFirstFrameId;
    cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
    for(Map* pMap : mpAtlas->GetAllMaps())
    {
        if(pMap->GetAllKeyFrames().size() > 0)
        {
            if(index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    int num_lost = 0;
    cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

    for(list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
    {
        if(index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else
        {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    cout << num_lost << " Frames set to lost" << endl;

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

vector<MapPoint*> Tracking::GetLocalMapMPS()
{
    return mvpLocalMapPoints;
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame)
{
    Map * pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mlRelativeFramePoses.begin(),lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        while(pKF->isBad())
        {
            pKF = pKF->GetParent();
        }

        if(pKF->GetMap() == pMap)
        {
            (*lit).rowRange(0,3).col(3)=(*lit).rowRange(0,3).col(3)*s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    cv::Mat Gz = (cv::Mat_<float>(3,1) << 0, 0, -IMU::GRAVITY_VALUE);

    cv::Mat twb1;
    cv::Mat Rwb1;
    cv::Mat Vwb1;
    float t12;

    while(!mCurrentFrame.imuIsPreintegrated())
    {
        usleep(500);
    }


    if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
    {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    }
    else
    {
        twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        t12 = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame.mpImuPreintegrated)
    {
        twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        t12 = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}


cv::Mat Tracking::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = Converter::tocvSkewMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}


void Tracking::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    const vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpLastKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpLastKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpLastKeyFrame->GetCameraCenter();

    const float &fx1 = mpLastKeyFrame->fx;
    const float &fy1 = mpLastKeyFrame->fy;
    const float &cx1 = mpLastKeyFrame->cx;
    const float &cy1 = mpLastKeyFrame->cy;
    const float &invfx1 = mpLastKeyFrame->invfx;
    const float &invfy1 = mpLastKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpLastKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF2 = vpKFs[i];
        if(pKF2==mpLastKeyFrame)
            continue;

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if((mSensor!=System::MONOCULAR)||(mSensor!=System::IMU_MONOCULAR))
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpLastKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpLastKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpLastKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpLastKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpLastKeyFrame->mb/2,mpLastKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpLastKeyFrame->UnprojectStereo(idx1);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpLastKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpLastKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpLastKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpLastKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,mpLastKeyFrame,mpAtlas->GetCurrentMap());

            pMP->AddObservation(mpLastKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpLastKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);
            nnew++;
        }
    }
    TrackReferenceKeyFrame();
}

void Tracking::NewDataset()
{
    mnNumDataset++;
}

int Tracking::GetNumberDataset()
{
    return mnNumDataset;
}

int Tracking::GetMatchesInliers()
{
    return mnMatchesInliers;
}

} //namespace ORB_SLAM
