#include "LaserDepthProjector.h"
#include "../utility/visualization.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

void LaserDepthProjector::updateRange(const std_msgs::msg::Float32::SharedPtr msg)
{
    double r = msg->data;

    if (width <= 0 || height <= 0) return;
    if (r < laser_min_range || r > laser_max_range) return;

    depth_map = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));
    depth_sigma_map = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));

    Eigen::Vector3d p_l(r, 0.0, 0.0);
    Eigen::Vector3d p_c = R_cl * p_l + t_cl;

    if (p_c.z() <= 0) return;

    int u = static_cast<int>(fx * p_c.x() / p_c.z() + cx);
    int v = static_cast<int>(fy * p_c.y() / p_c.z() + cy);

    if (u < 0 || u >= width || v < 0 || v >= height) return;

    double xn = (u - cx) / fx;
    double yn = (v - cy) / fy;
    double tan_theta2 = xn * xn + yn * yn;

    double k = 3.0;
    double sigma = laser_noise_std * (1.0 + k * tan_theta2);
    sigma = std::min(sigma, 5.0 * laser_noise_std);

    depth_map.at<float>(v, u) = static_cast<float>(p_c.z());
    depth_sigma_map.at<float>(v, u) = static_cast<float>(sigma);

    ready = true;
}

bool LaserDepthProjector::getDepth(double u_norm, double v_norm, double &depth_out, double &sigma_out)
{
    if (!ready)
        return false;

    int u = fx * u_norm + cx;
    int v = fy * v_norm + cy;

    if (u < 0 || u >= width || v < 0 || v >= height)
        return false;

    float d = depth_map.at<float>(v, u);
    float s = depth_sigma_map.at<float>(v, u);

    if (d <= 0)
        return false;

    depth_out = d;
    sigma_out = s;

    if (sigma_out > 3*laser_noise_std || depth_out < laser_min_range || depth_out > laser_max_range)
        return false;

    return true;
}

void LaserDepthProjector::loadLRFConfig(std::string path1, std::string path2)
{
    cv::FileStorage fs(path1, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        throw std::runtime_error("Failed to open LRF config file: " + path1);
    }

    std::vector<double> r, t;

    double sampling_rate_tmp;
    double noise_tmp;
    double res_tmp;
    double ds_tmp;
    double min_range_tmp;
    double max_range_tmp;

    fs["lrf_to_camera_rotation"] >> r;
    fs["lrf_to_camera_translation"] >> t;
    fs["sampling_rate"] >> sampling_rate_tmp;
    fs["laser_noise_std"] >> noise_tmp;
    fs["laser_resolution"] >> res_tmp;
    fs["laser_downsample"] >> ds_tmp;
    fs["laser_min_range"] >> min_range_tmp;
    fs["laser_max_range"] >> max_range_tmp;

    fs.release();

    // Validate sizes
    if (r.size() != 9 || t.size() != 3)
    {
        throw std::runtime_error("Invalid lrf_to_camera extrinsics size");
    }

    // Assign extrinsics
    R_cl <<
        r[0], r[1], r[2],
        r[3], r[4], r[5],
        r[6], r[7], r[8];

    t_cl << t[0], t[1], t[2];

    // Assign scalars
    sampling_rate   = sampling_rate_tmp;
    laser_noise_std = noise_tmp;
    laser_resolution = res_tmp;
    laser_downsample = ds_tmp;
    laser_min_range  = min_range_tmp;
    laser_max_range  = max_range_tmp;

    cv::FileStorage fs2(path2, cv::FileStorage::READ);

    if (!fs2.isOpened())
    {
        throw std::runtime_error("Failed to open Camera config file: " + path2);
    }

    double fx_tmp;
    double fy_tmp;
    double cx_tmp;
    double cy_tmp;
    double width_tmp;
    double height_tmp;

    cv::FileNode n = fs2["projection_parameters"];
    n["fx"] >> fx_tmp;
    n["fy"] >> fy_tmp;
    n["cx"] >> cx_tmp;
    n["cy"] >> cy_tmp;
    fs2["image_width"] >> width_tmp;
    fs2["image_height"] >> height_tmp;

    fs2.release();

    // Assign scalars
    fx   = fx_tmp;
    fy = fy_tmp;
    cx = cx_tmp;
    cy = cy_tmp;
    width = width_tmp;
    height = height_tmp;

    std::cout << "[LRF] Config loaded successfully\n";
}