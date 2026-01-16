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
        double sigma = laser_noise_std;

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