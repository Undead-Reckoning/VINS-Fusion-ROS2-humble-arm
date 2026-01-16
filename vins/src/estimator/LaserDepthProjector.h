#pragma once
 
#include <thread>
#include <mutex>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"

#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_ERROR RCUTILS_LOG_ERROR

class LaserDepthProjector
{
public:
    cv::Mat depth_map;   // CV_32FC1, meters
    Matrix3d R_cl;       // camera ← laser
    Vector3d t_cl;

    int width, height;
    double fx, fy, cx, cy;

    bool ready = false;

    void setExtrinsics(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
    {
        R_cl = R;
        t_cl = t;
    }

    void updatePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
    bool getDepth(double u_norm, double v_norm, double &depth_out);
};