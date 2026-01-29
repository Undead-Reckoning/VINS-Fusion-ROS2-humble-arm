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


class MagnetometerFactor : public ceres::SizedCostFunction<3, 7>
{
  public:
    MagnetometerFactor() = delete;
    /**
     * Construct a MagnetometerFactor
     * @param x_meas measured magnetic field (microT)
     * @param y_meas
     * @param z_meas
     */
    MagnetometerFactor(double x_meas_, double y_meas_, double z_meas_, double sigma)
      : x_meas(x_meas_), y_meas(y_meas_), z_meas(z_meas_)
    {
      if (sigma <= 0)
        sigma = 1.0;
      sqrt_info = 1.0 / sigma;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // pose parameterization in this codebase: [px, py, pz, qx, qy, qz, qw]
        const double qz = parameters[0][5];

        Eigen::Quaterniond q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Matrix3d Rwb = q.toRotationMatrix();


        //residual
        mag_inert_norm = Eigen::Vector3d(0,1,0).normalized(); //Magnetic North, assumed temp
        m_meas = Eigen::Vector3d(x_meas, y_meas, z_meas);
        //m_meas_norm = Eigen::Vector3d(meas./(x_meas**2 + y_meas**2 + z_meas**2));
        m_meas_norm = m_meas.normalized();

        //transpose
        Eigen::Map<Eigen::Vector3d> res(residuals);
        residuals = (m_meas_norm - Rwb.transpose() * mag_inert_norm) * sqrt_info;
             // (jacobians && jacobians[0])
       if (jacobians && jacobians[0]){
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_mag(jacobians[0]);
            jacobian_pose_mag.setZero();
            //Eigen::Matrix3d jacobian_rotation = -Rwb.transpose() * skew(mag_inert_norm);
            Eigen::Matrix3d jacobian_mag = -Rwb.transpose() * mag_inert_norm;
            jacobian_pose_mag.block<3,3>(0,3) = jacobian_mag; //maybe remove prev negative, add neg skew() around here
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

    double x_meas;    // measured magnetic field (microT)
    double y_meas;
    double z_meas;
    double sqrt_info; // 1 / sigma