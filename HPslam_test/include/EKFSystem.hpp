//
//  EKFSystem.hpp
//  ekf_final
//
//  Created by 谭智丹 on 17/2/8.
//  Copyright © 2017年 谭智丹. All rights reserved.
//

#ifndef EKFSystem_hpp
#define EKFSystem_hpp

#include "EigenHelper.h"
#include "Camera.hpp"
#include "CamPose.hpp"
#include "Feature.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "maker_binocular.h"

using namespace cv;
typedef Eigen::Matrix<double,3,4> Mat3x4d;
typedef Eigen::Vector4d v4d;
typedef Eigen::Vector3d v3d;

class EKFSystem
{
public:
    
    EKFSystem(bool half_res, int n_camx);
    
    int imu_begin;
    int grt_begin;
    int first_frame;
    int total_frames;
    
    EgQuatd q_Sc;
    EgQuatd q_Cs;
    EgM3d   R_Cs;
    
    std::vector<EgV3d> map_points;
    std::vector<EgV3d> trajectory;
	std::vector<int> inlierHistory;
    
    cv::Mat K_left,K_right,D_left,D_right,R_wl,R_wr,t_wl,t_wr,T_wl,T_wr;
    Eigen::Matrix3d eK_l,eK_r,eR_wl,eR_wr,eR_lw,eR_rw;
    Eigen::Vector3d et_wl,et_wr,et_lw,et_rw;
    Mat3x4d eT_wl,eT_wr,eT_lw,eT_rw,eP_lw,eP_rw;

    void LoadParameters(cyusb_handle *handle1,std::string& dir);
    
    void Initialize(EgV3d& init_p, EgV3d& init_g, EgV3d& init_v, EgV3d& init_ba, EgV3d& init_bw, double img_time);
    
    void ManageMap(Mat& I,Mat&Ir, int iStep);
    
    void Propagate(std::vector<double> &vts, std::vector<EgV3d>& wms, std::vector<EgV3d>& ams, double img_time);
    
    void SearchMatches(Mat& I);
    
    void BigUpdate();
    
    void DrawOnImage(Mat &I);
    
    void GetActivePoints(std::vector<EgV3d>& active_points);

    void estimatedV(cv::Mat lastimg_l,cv::Mat lastimg_r,cv::Mat currimg_l,Eigen::Vector3d &V);

	bool Supervise(float maxVelocity);
	
	void TryToFindLostLocation();
	
	void Reset();
    
public:

	double ThisTimeStamp;

	double LastTimeStamp;
    
    int NcamX;
    
    bool bHalfRes;
    
    int NWanted;
    
    int fast_threshold;
    
    double sigma_gyro;
    double sigma_accel;
    double sigma_pixel;
    double imu_rate;
    double deltat;
    double gravity_magnitude;
    
    Camera Cam;

    int imWidth, imHeight;
    double fx, fy, cx, cy, k1, k2, p1, p2, k3,baseline;
    
    double ncc_threshold;
    double zncc_threshold;
    double ssd_threshold;
    
    int hps_ini;
    int hps_mat;
    int edge_band;
    double max_angle;
    
    double median_inverse_depth;
    double max_inverse_depth;
    double min_inverse_depth;
    double prior_inverse_depth;
    bool well_estimated;
    
    Eigen::VectorXd x_k_k;
    Eigen::MatrixXd P_k_k;
    
    std::vector<Feature> features_info;
    std::vector<CamPose> poses_info;
    std::vector<int> entries[2];
    
    EgV3d LockedPosition;
    
private:
    
    void SearchNewFeatures(Mat& I,int numWanted, std::vector<Point2f>& new_pts);

    void Triangle(std::vector<Point2f>&last_kp,std::vector<Point2f>&curr_kp,std::vector<double>&depth);

    void Triangle3d(std::vector<Point2f>&last_kp,std::vector<Point2f>&curr_kp,std::vector<Point3f>&mappoints);

    Eigen::MatrixXd caculateFundamentalMatrix(Eigen::Matrix3d eK_l,Eigen::Matrix3d eK_r,Eigen::Matrix3d eR_lw,Eigen::Matrix3d eR_rw,Eigen::Vector3d et_lw,Eigen::Vector3d et_rw);

    void UDpoints(std::vector<Point2f>&last_kp);

    void UDpoints(std::vector<Point2f>&last_kp,std::vector<Point2f>&curr_kp);

    double caculateDistanceByF(Point2f lastKP,Point2f currKP,Eigen::MatrixXd F);

    bool TriangulateDLT(const Mat3x4d &pose1,const Mat3x4d &pose2,const Eigen::Vector2d &pt1,const Eigen::Vector2d &pt2,Eigen::Vector4d& output);
    
   void AddToFilter(std::vector<Point2f>& newFeatures, std::vector<double> initial_rho, std::vector<double> std_rho);
    
    void AddToInfos(std::vector<Point2f>& newFeatures, Mat& I, int iStep);
    
    void DeleteFeatures();
    
    void UpdateEntries();
    
    int UpdateFeaturesInfo();
    
    void EstimateMedianInverseDepth();
    
    void Calculate_little_hs();
    
    void Calculate_big_Hs();
    
    void Calculate_Ss();
    
    void NccMatching(Mat& I);
    
    void OnePointRansac(double palosf);
    
    void OnePointUpdateState( Eigen::VectorXd &xNew, int iFeature);
    
    void ComeOnGuys(Eigen::VectorXd &xNew, std::vector<int>& posis_cp, std::vector<int>& iInliers);
    
    void EkfUpdateLowInnos();
    
    void EkfUpdateHighInnos();
    
    void EkfUpdate(std::vector<int>& iFeatures);

	void EkfUpdateUsingGravityAlone(EgV3d& gs);

	void EkfUpdateWithGravity(EgV3d& gs);
    
    void RescueHighInnos();
    
    void PreIntegrate(std::vector<double>& ts,
                      std::vector<EgV3d>& wms,
                      std::vector<EgV3d>& ams,
                      EgV3d& pp, EgQuatd& qq, EgV3d& vv,
                      Eigen::Matrix<double, 10, 10> &RR);
    
    
    
};

#endif /* EKFSystem_hpp */
