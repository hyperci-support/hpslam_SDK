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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include<pangolin/pangolin.h>

#include<mutex>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include "Map.h"

using namespace cv;
using namespace std;

namespace YX_SLAM
{

class MapDrawer
{
public:
    MapDrawer(YX_SLAM::Map* _mp, const string &strSettingPath);
    MapDrawer(const string &strSettingPath);

    //Map* mpMap;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    //void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    void AddPose(const cv::Mat& _Twc);
    void AddFrame(const cv::Mat& _im);
    cv::Mat DrawFrame();
private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
    
    vector<cv::Mat> poseList;
    vector<cv::Point3f> mappointList;
    
    cv::Mat im;
    YX_SLAM::Map* mpMap;
    //int fCount;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
