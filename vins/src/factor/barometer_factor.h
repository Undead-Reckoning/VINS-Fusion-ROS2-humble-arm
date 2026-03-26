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
      if (sigma <= 0) {
        ROS_WARN("BarometerFactor: Invalid sigma (%.3f). Setting to default 1.0 m", sigma);
        sigma = 1.0;
      }
      sqrt_info = 1.0 / sigma;
      //printf("[BARO_CTOR] BarometerFactor created: z_meas=%.3f m, sigma=%.3f m, sqrt_info=%.6f\n", z_meas, sigma, sqrt_info);
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // pose parameterization in this codebase: [px, py, pz, qx, qy, qz, qw]
        const double pz = parameters[0][2];

        // 1D residual: body z minus measured altitude
        residuals[0] = (pz - z_meas) * sqrt_info;
        //printf("[BARO_EVAL] Evaluate: pz=%.3f m, z_meas=%.3f m, residual=%.6f\n", pz, z_meas, residuals[0]);

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
        printf("[BARO_SETMEAS] Measurement updated: z=%.3f m, sigma=%.3f m, sqrt_info=%.6f\n", z, sigma, sqrt_info);
    }

    double z_meas;    // measured altitude (m)
    double sqrt_info; // 1 / sigma
  
};