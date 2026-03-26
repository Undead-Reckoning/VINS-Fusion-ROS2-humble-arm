#include "LaserDepthProjector.h"
#include "../utility/visualization.h"

/*
    MODIFIED    
    Undead Reckoning
    Date: 01/21/26
    By: Quinn Levinson
*/

void LaserDepthProjector::updateRange(const sensor_msgs::msg::Range::SharedPtr msg)
{
    const double r = msg->range;

    if (r < laser_min_range || r > laser_max_range)
        return;

    const double t_now =
        msg->header.stamp.sec +
        1e-9 * msg->header.stamp.nanosec;

    std::lock_guard<std::mutex> lock(buffer_mutex);

    range_buffer.push_back({t_now, r});

    while (!range_buffer.empty() &&
           (t_now - range_buffer.front().t) > filter_window_sec)
    {
        range_buffer.pop_front();
    }

    if (range_buffer.empty())
        return;

    double weighted_sum = 0.0;
    double weight_sum   = 0.0;

    const double alpha = 4.0;

    for (const auto &m : range_buffer)
    {
        double age = t_now - m.t;
        double w = std::exp(-alpha * age);

        weighted_sum += w * m.r;
        weight_sum   += w;
    }

    if (weight_sum <= 1e-6)
        return;

    const double r_filtered = weighted_sum / weight_sum;

    Eigen::Vector3d p_l(0.0, 0.0, r_filtered);
    p_c = R_cl * p_l + t_cl;

    if (p_c.z() <= 0)
        return;

    ready = true;

    //printf("Filtered Depth: %f\n", p_c.z());
}

bool LaserDepthProjector::getDepth(double u_norm, double v_norm, double &depth_out, double &sigma_out)
{
    if (!ready) return false;

    Eigen::Vector3d ray(u_norm, v_norm, 1.0);
    //ray.normalize();

    double denom = plane_normal.dot(ray);
    if (fabs(denom) < 1e-6)
        return false;

    double t = plane_normal.dot(p_c) / denom;
    if (t <= 0)
        return false;

    depth_out = t * ray.z();

    sigma_out = laser_noise_std * (1.0 + u_norm*u_norm + v_norm*v_norm);

    return true;
}

void LaserDepthProjector::loadLRFConfig(std::string path)
{

    cv::FileStorage fs(path, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        throw std::runtime_error("Failed to open LRF config file: " + path);
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

    //std::cout << "[LRF] Config loaded successfully\n";
}

/*
    END MODIFIED
*/