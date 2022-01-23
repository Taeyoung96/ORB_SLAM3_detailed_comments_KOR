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


#ifndef SYSTEM_H
#define SYSTEM_H

#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Config.h"


namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class FrameDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4
    };

    // File type
    enum eFileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string(), const string &strLoadingFile = std::string());

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    /* !
    * @brief EuRoC dataset에 대한 Trajectory를 저장하는 함수
    * @call mono_euroc.cc / mono_inertial_euroc.cc / stereo_euroc.cc / stereo_inertial_euroc.cc
    * @param filename 저장될 파일 이름    
    * @return None
    */
    void SaveTrajectoryEuRoC(const string &filename);

    /* !
    * @brief EuRoC dataset에 대한 Keyframe Trajectory를 저장하는 함수
    * @call mono_euroc.cc / mono_inertial_euroc.cc / stereo_euroc.cc / stereo_inertial_euroc.cc
    * @param filename 저장될 파일 이름    
    * @return None
    */
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);


    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    /* !
     * @brief KITTI dataset format으로 trajectory 저장
     * @call ros_stereo.cc에서 Shutdown()함수 실행 후 사용
     * @param Trajectory 출력 파일(.txt)
     * @return None
    */
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    // 가장 최근에 처리된 프레임의 정보를 알아옵니다.
    // TrackMonocular (or stereo or RGBD)를 사용한 다음에 함수들을 호출할 수 있습니다.

    /* !
     * @brief Tarcking의 상태를 알기 위해서 사용하는 함수
     * @param None
     * @return Tracking 상태 (Tracking.h의 enum eTrackingState 참고)
    */
    int GetTrackingState();
    
    /* !
     * @brief key frame의 map points들의 정보를 알기 위해 사용하는 함수,  
     *  key frame은 TrackMonocular (or stereo or RGBD)에서 결정
     * @param None
     * @return Map points들을 담고 있는 vector
    */
    std::vector<MapPoint*> GetTrackedMapPoints();

    /* !
     * @brief Undistorted된 key frame의 key points들의 정보를 알기 위해 사용하는 함수,  
     *  key frame은 TrackMonocular (or stereo or RGBD)에서 결정
     * @param None
     * @return key points들을 담고 있는 vector
    */
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging

    /* !
     * @brief IMU initialization 정보를 통해서 Timestamp 값을 얻어내기 위한 함수
     * @param None
     * @return Timestamp 값
    */
    double GetTimeFromIMUInit();

    /* !
     * @brief LOST상태인지 아닌지, 판단하기 위한 함수
     * @param None
     * @return LOST 상태이면 true, 그렇지 않으면 false
    */
    bool isLost();

    /* !
     * @brief Finish인지 확인하는 함수 (Time stamp를 활용)
     * @param None
     * @return Finish 상태이면 true, 그렇지 않으면 false
    */
    bool isFinished();

    /* !
     * @brief Dataset(Map)을 바꾸는 함수 : 여러 Sequence를 사용할 때
     * @param None
     * @return None
    */
    void ChangeDataset();

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);

    void InsertTrackTime(double& time);
#endif

private:

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Atlas structure that stores the pointers to all KeyFrames and MapPoints.
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
