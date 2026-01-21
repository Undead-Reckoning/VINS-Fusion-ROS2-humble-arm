#pragma once
#include <rcpputils/asserts.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_ERROR RCUTILS_LOG_ERROR


class BarometerFactor : public ceres::SizedCostFunction<1, 7>
{
  public:
    BarometerFactor() = delete;
    /**
     * Construct a BarometerFactor
     * @param z_meas measured altitude (meters)
     * @param sigma standard deviation of measurement (meters)
     */
    BarometerFactor(double z_meas_, double sigma)
      : z_meas(z_meas_)
    {
      if (sigma <= 0)
        sigma = 1.0;
      sqrt_info = 1.0 / sigma;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // pose parameterization in this codebase: [px, py, pz, qx, qy, qz, qw]
        const double pz = parameters[0][2];

        // 1D residual: body z minus measured altitude
        residuals[0] = (pz - z_meas) * sqrt_info;

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();
            jacobian_pose(0, 2) = sqrt_info;
        }

        return true;
    }

    void setMeasurement(double z, double sigma)
    {
        z_meas = z;
        if (sigma <= 0)
            sigma = 1.0;
        sqrt_info = 1.0 / sigma;
    }

    double z_meas;    // measured altitude (m)
    double sqrt_info; // 1 / sigma