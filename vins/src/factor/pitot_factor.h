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


class PitotFactor : public ceres::SizedCostFunction<1, 7, 9>
{
  public:
    PitotFactor() = delete;
    /**
     * Construct a PitotFactor
     * @param vx_meas measured x body frame velocity (meters / second)
     * @param sigma standard deviation of measurement (meters)
     */
    PitotFactor(double vx_meas_, double sigma)
      : vx_meas(vx_meas_)
    {
      if (sigma <= 0)
        sigma = 1.0;
      sqrt_info = 1.0 / sigma;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // pose parameterization in this codebase: [px, py, pz, qx, qy, qz, qw]
        const double px = parameters[0][0];

        // 1D residual: body x velocity minus measured velocity 
        residuals[0] = (px - vx_meas) * sqrt_info;

        if (jacobians && jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();
            jacobian_pose(0, 2) = sqrt_info;
        }

        return true;
    }

    void setMeasurement(double vx, double sigma)
    {
        vx_meas = vx;
        if (sigma <= 0)
            sigma = 1.0;
        sqrt_info = 1.0 / sigma;
    }

    double vx_meas;    // measured altitude (m)
    double sqrt_info; // 1 / sigma