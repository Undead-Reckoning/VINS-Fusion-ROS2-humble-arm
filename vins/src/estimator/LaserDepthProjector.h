#pragma once

#include <mutex>

#include <opencv2/core.hpp>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LaserDepthProjector
{
public:
    cv::Mat depth_map;   // CV_32FC1, meters
    cv::Mat depth_sigma_map;
    Matrix3d R_cl;       // camera ← laser
    Vector3d t_cl;
    double sampling_rate;
    double laser_noise_std;
    double laser_resolution;
    double laser_downsample;
    double laser_min_range;
    double laser_max_range;

    int width, height;
    double fx, fy, cx, cy;

    bool ready = false;

    void setExtrinsics(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
    {
        R_cl = R;
        t_cl = t;
    }

    void updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    bool getDepth(double u_norm, double v_norm, double &depth_out, double &sigma_out);
};