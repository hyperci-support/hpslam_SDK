//
//  IMUFilter.hpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/17.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#ifndef IMUFilter_hpp
#define IMUFilter_hpp

#include <stdio.h>
#include <iostream>
#include "EigenHelper.h"

class IMUFilter
{

public:
	IMUFilter(int freq, double gyroMeasErr, double gyroAlpha, double gyroAbsThresh, double accelAbsThresh, EgV3d& ba, EgV3d& bg);

	void SetAccelBias(EgV3d &ba);
	void SetGyroBias(EgV3d &bg);

	EgV3d GetAccelBias();
	EgV3d GetGyroBias();
	
	void SetQuaternion(EgQuatd& q);
	EgQuatd GetQuaternion();

	bool HandleData(EgV3d& wm, EgV3d& am, double deltat);

	bool HandleDatas(std::vector<EgV3d>& wms, std::vector<EgV3d>& ams, double deltat);

	bool GyroPretreatment(EgV3d& wm);

	void MadgwickFusion(EgV3d& wm, EgV3d& am, double deltat);

	bool ResetQfromA(EgV3d& am);

	EgV3d Quat2YawPitchRoll(EgQuatd &qsw);
	
private:

	// Frequncy (Hz)
	int Freq;
	int StaticWindow;

	// Biases
	EgV3d AccelBias;
	EgV3d GyroBias;

	// Buffer size
	int BufferCapacity;
	
	// For filtering accel readings
	std::vector<EgV3d> AccelFilterBuffer;

	// For filtering gyro readings
	std::vector<EgV3d> GyroFilterBuffer;
	EgV3d LastGyro;
	int GyroCount;
	EgV3d GyroSum;
	double GyroAlpha;

	// Stationary threshold
	double GyroAbsThresh;
	double GyroSigmaThresh;
	double AccelAbsThresh;
	double AccelSigmaThresh;

	// Madgwick filter settings
	double GyroMeasError;
	double Beta;

	// Well-initialized
	bool QuatIsGood;
	
	// Madgwick filter variables
	double SEq_1;
	double SEq_2;
	double SEq_3;
	double SEq_4;

};

#endif /* IMUFilter_hpp */
