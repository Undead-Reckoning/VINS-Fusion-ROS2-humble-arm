#include "LaserDepthProjector.h"
#include "../utility/visualization.h"

void LaserDepthProjector::updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    depth_map = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));
    depth_sigma_map = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    for (auto &pt : cloud.points)
    {
        // Laser frame
        Vector3d p_l(pt.x, pt.y, pt.z);

        // Transform to camera frame
        Vector3d p_c = R_cl * p_l + t_cl;

        if (p_c.z() < laser_min_range || p_c.z() > laser_max_range)
            continue;

        // Project to pixel
        int u = fx * p_c.x() / p_c.z() + cx;
        int v = fy * p_c.y() / p_c.z() + cy;

        if (u < 0 || u >= width || v < 0 || v >= height)
            continue;

        float &d = depth_map.at<float>(v, u);
        float &s = depth_sigma_map.at<float>(v, u);

        double depth = p_c.z();

        // Normalized image coordinates
        double xn = (u - cx) / fx;
        double yn = (v - cy) / fy;
        double tan_theta2 = xn * xn + yn * yn;

        // Tunable coefficient
        double k = 3.0;

        // Edge-aware sigma
        double sigma = laser_noise_std * (1.0 + k * tan_theta2);
        sigma = std::min(sigma, 5.0 * laser_noise_std);

        if (d == 0 || depth < d)
        {
            d = depth;
            s = sigma;
        }
        else
        {
            float alpha = 0.6f;
            d = alpha * d + (1 - alpha) * depth;
            s = alpha * s + (1 - alpha) * sigma;
        }
    }

    cv::medianBlur(depth_map, depth_map, 3);

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

    cv::FileStorage fs(path2, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        throw std::runtime_error("Failed to open Camera config file: " + path2);
    }

    double fx_tmp;
    double fy_tmp;
    double cx_tmp;
    double cy_tmp;
    double width_tmp;
    double height_tmp;

    n = fs["projection_parameters"];
    n["fx"] >> fx_tmp;
    n["fy"] >> fy_tmp;
    n["cx"] >> cx_tmp;
    n["cy"] >> cy_tmp;
    fs["image_width"] >> width_tmp;
    fs["image_height"] >> height_tmp;

    fs.release();

    // Assign scalars
    fx   = fx_tmp;
    fy = fy_tmp;
    cx = cx_tmp;
    cy = cy_tmp;
    width = width_tmp;
    height = height_tmp;

    std::cout << "[LRF] Config loaded successfully\n";
}