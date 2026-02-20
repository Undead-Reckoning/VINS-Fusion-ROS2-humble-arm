#pragma once
#include <rcpputils/asserts.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"
//#include "integration_base.h" // may not need this-> check

#include <ceres/ceres.h>

#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_ERROR RCUTILS_LOG_ERROR

/**
     * @brief Pitot_tube factor with wind estimation (constant wind, need to publish conditions before flight) 
     * @param vx_meas measured vx in body frame  (meters / second)
     * @param sigma standard deviation of measurement (meters)
     * 
     * Parameters for this method
     * < 1, 7, 9>
     * 1: residual dimension (just have one)
     * 7: pose, position and quaternion [px, py, pz, qx, qy, qz, qw]
     * 9: state, velocity and bias state [vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
     * 
     * 
     * Note on variable naming: vx is body frame x velocity, q_wb is rotation from world to body frame, v_w is velocity in world frame
     * V_air_body is air velocity in body frame
     *  
     * 
     * vx_body = (R_b_w * (v_ground_world - wind_world)).x()
     *   residual = (vx_meas - vx_body) / sigma
     */
class PitotFactor : public ceres::SizedCostFunction<1, 7, 9> // <1 residual, 7 for the quaternion and x,y,z, 9 state vector (need velocity from this)>
{
  public:
    PitotFactor() = delete;
    PitotFactor(double vx_meas, double sigma) // 
      : vx_meas(vx_meas), wind_velocity(0,0,0)
    {
      if (sigma <= 0)
        sigma = 1.0;
      sqrt_info = 1.0 / sigma;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // Getting states from input parameters
        const double px = parameters[0][0], py = parameters[0][1], pz = parameters[0][2];
        Eigen::Vector3d position(px, py, pz); // position of body in world frame
        Eigen::Quaterniond q_wb(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]); // qw, qx, qy, qz for rotation from worldto body frame 
        Eigen::Vector3d v_ground_world(parameters[1][0], parameters[1][1], parameters[1][2]); // velocity of body in world frame (ground speed)
        //Eigen::Vector3d wind_world(parameters[2][0], parameters[2][1], parameters[2][2]); // wind in world frame
        
        // Airspeed calculation
        // We are using wind_velocity as a CONSTANT member variable not from parameters 
        Eigen::Vector3d v_air_world = v_ground_world - wind_velocity; // air velocity in world frame
        Eigen::Vector3d v_air_body = q_wb.inverse() * v_air_world; /// air velocity in body frame
        //Eigen::Vector3d v_body = q_wb.inverse() * v_ground_world; /// air velocity in body frame (assuming no wind)
       
       // THIS DOES NOT ACCOUNT FOR SIDESLIP
        double vx_pred = v_air_body.x(); // predicted body frame x velocity with wind 

        // 1D residual: body x velocity minus measured velocity 
        residuals[0] = (vx_meas - vx_pred) * sqrt_info;

        
        // NEED: Jacobian wrt to pose
        if (jacobians){

          if (jacobians[0])  
          {
            // Jacobian wrt to pose
            
            //Jacobian is not dependent on position, so first three columns are zero
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();
            // Position derivatives are zero
            jacobian_pose.block<1, 3>(0, 0).setZero();

            Eigen::Matrix3d R_b_w = q_wb.inverse().toRotationMatrix();
            Eigen::Matrix3d skew_V_air_world = Utility::skewSymmetric(v_air_world);
            Eigen::Vector3d d_Vx_d_rotation = -(R_b_w * skew_V_air_world).row(0).transpose();
              
            jacobian_pose.block<1, 3>(0, 3) = -sqrt_info * d_Vx_d_rotation.transpose() * 0.5;
            jacobian_pose(0, 6) = 0.0;
          }
          
          // Jacobian wrt to velocity
          if (jacobians[1])  
          {
            Eigen::Map<Eigen::Matrix<double, 1, 9, Eigen::RowMajor>> jacobian_speedbias(jacobians[1]);
            jacobian_speedbias.setZero();
            Eigen::Matrix3d R_b_w = q_wb.inverse().toRotationMatrix();
            jacobian_speedbias.block<1, 3>(0, 0) = -sqrt_info * R_b_w.row(0);

            // Bias doesn't have affect on the airspeed 
            jacobian_speedbias.block<1, 3>(0, 3).setZero();
            jacobian_speedbias.block<1, 3>(0, 6).setZero();
          }
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

    private:
    double vx_meas;    // measured x-velocity in body frame (m)
    Eigen::Vector3d wind_velocity; // wind velocity in world frame (m/s)
    double sqrt_info; // 1 / sigma
  };