#pragma once
#include <rcpputils/asserts.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

#include <fstream>

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
    MagnetometerFactor(double x_meas_, double y_meas_, double z_meas_, double sigma, Eigen::Matrix3d R_MV_)
      : x_meas(x_meas_), y_meas(y_meas_), z_meas(z_meas_), R_MV(R_MV_)
    {
      if (sigma <= 0)
        sigma = 1.0;
      sqrt_info = 1.0 / sigma;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // pose parameterization in this codebase: [px, py, pz, qx, qy, qz, qw]
        //const double qz = parameters[0][5];
        //cout << parameters[0][6] << endl;
        Eigen::Quaterniond q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        Eigen::Matrix3d Rwb = q.toRotationMatrix();


        //residual 
        Eigen::Vector3d mag_field = Eigen::Vector3d(0.0, 1.0, 0.0);
        Eigen::Vector3d mag_field_norm = mag_field.normalized();
        
        Eigen::Vector3d m_meas = Eigen::Vector3d(x_meas, y_meas, z_meas);
        Eigen::Vector3d m_meas_norm = m_meas.normalized();
        cout << "meas " << m_meas << endl;
        //file for testing
        //std::ofstream measfile("/tmp/TESTING_WF_MEAS_7_on.csv", std::ios::app);
        //measfile << x_meas << " " << y_meas << " " << z_meas << std::endl;
        //measfile.flush();
        //measfile.close();

        //actual = measured, predicted = m_b (following eq)
        Eigen::Vector3d m_VINS = R_MV.transpose() * mag_field_norm;
        Eigen::Vector3d m_pred = (Rwb).transpose() * m_VINS;
        Eigen::Map<Eigen::Vector3d> res(residuals);
        
        res = (m_meas_norm - m_pred) * sqrt_info;
        //cout << "RESIDUALS: " << (m_meas - Rwb.transpose() * mag_inert) << endl;
             // (jacobians && jacobians[0])
       if (jacobians && jacobians[0]){
            Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose_mag(jacobians[0]);
            jacobian_pose_mag.setZero();
            //Eigen::Vector3d mag_body = Rwb.transpose() * mag_field_norm; //here
            Eigen::Vector3d mag_body = Rwb.transpose() * m_VINS; 
            jacobian_pose_mag.block<3,3>(0,3) = -Utility::skewSymmetric(mag_body) * sqrt_info;
            //jacobian_pose_mag *= sqrt_info;
       }


        return true;
    }
    void setMeasurement(double z, double sigma)
    {
      cout << "SETMEASUREMENT" << endl;
        //z_meas = z;
        if (sigma <= 0)
            sigma = 1.0;
        sqrt_info = 1.0 / sigma;
    }

    double x_meas;    // measured magnetic field (microT)
    double y_meas;
    double z_meas;
    double sqrt_info; // 1 / sigma
    Eigen::Matrix3d R_MV;
    //double m_meas_norm;
    //double m_meas;
    //double mag_inert_norm;
  };