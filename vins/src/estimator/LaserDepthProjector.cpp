#include "LaserDepthProjector.h"
#include "../utility/visualization.h"

void LaserDepthProjector::updatePointCloud(
        const sensor_msgs::PointCloud2ConstPtr& msg)
{
    depth_map = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    for (auto &pt : cloud.points)
    {
        // Laser frame
        Vector3d p_l(pt.x, pt.y, pt.z);

        // Transform to camera frame
        Vector3d p_c = R_cl * p_l + t_cl;

        if (p_c.z() <= 0.1) continue;

        // Project to pixel
        int u = fx * p_c.x() / p_c.z() + cx;
        int v = fy * p_c.y() / p_c.z() + cy;

        if (u < 0 || u >= width || v < 0 || v >= height)
            continue;

        float &d = depth_map.at<float>(v, u);

        if (d == 0 || p_c.z() < d)
            d = p_c.z();
    }

    ready = true;
}

bool LaserDepthProjector::getDepth(double u_norm, double v_norm, double &depth_out)
{
    if (!ready)
        return false;

    int u = fx * u_norm + cx;
    int v = fy * v_norm + cy;

    if (u < 0 || u >= width || v < 0 || v >= height)
        return false;

    float d = depth_map.at<float>(v, u);

    if (d <= 0 || std::isnan(d))
        return false;

    depth_out = d;
    return true;
}