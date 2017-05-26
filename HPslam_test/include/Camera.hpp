//
//  Camera.hpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/8.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#ifndef Camera_hpp
#define Camera_hpp

#include <opencv2/opencv.hpp>
#include "EigenHelper.h"

using namespace cv;

class Camera
{
public:
    void SetIntrinsics(int width, int height, double focalx, double focaly, double ppx, double ppy);

	void SetK(double fx_, double fy_, double cx_, double cy_);

	void SetDistortion(double k1, double k2, double p1, double p2, double k3);

	void UndistortPts(std::vector<Point2f>& vPtsDi, std::vector<Point2f>& vPtsUn);

	Point2f UndistortOnePt(Point2f pt);

	EgV2d ProjectAndDistort(double x, double y);    

    int nCols;
    int nRows;
    double fx;
    double fy;
    double Cx;
    double Cy;
	double mk1;
	double mk2;
	double mp1;
	double mp2;
	double mk3;

	Mat K;
	Mat distCoeffs;
};

#endif /* Camera_hpp */
