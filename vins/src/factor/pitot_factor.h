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


class PitotFactor : public ceres::SizedCostFunction<1, 7, 9, 3> // <1 residual, 7 for the quaternion and x,y,z, 9 state vector (need velocity from this), 3 wind(wx,wy,wz)>
{
  public:
    PitotFactor() = delete;
    /**
     * Construct a PitotFactor
     * @param vx_meas measured x body frame velocity (meters / second)
     * @param sigma standard deviation of measurement (meters)
     * 
     * Parameters for this method
     * < 1, 7, 9, 3>
     * 1: residual dimension (just have one)
     * 7: pose, position and quaternion [px, py, pz, qx, qy, qz, qw]
     * 9: state, velocity and bias state [vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
     * 3: wind [wx, wy, wz]
     * 
     * 
     * Note on variable naming: vx is body frame x velocity, q_wb is rotation from world to body frame, v_w is velocity in world frame
     * V_air_body is air velocity in body frame
     *  
     */
    PitotFactor(double vx_meas_, double sigma) // 
      : vx_meas(vx_meas_)
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
        Eigen::Vector3d wind_world(parameters[2][0], parameters[2][1], parameters[2][2]); // wind in world frame
        
        // Airspeed calculation
        Eigen::Vector3d v_air_world = v_ground_world - wind_world; // air velocity in world frame
        Eigen::Vector3d v_air_body = q_wb.inverse() * v_air_world; /// air velocity in body frame
       

       ///IS THE BODY FRAME X VELOCITY THE FIRST ELEMENT OF THIS VECTOR?
       //OR IS THIS THE NORM OF THE VECTOR? Pretty sure this is correct but gotta verify 
        double vx_pred = v_air_body.x(); // predicted body frame x velocity with wind 
        

        
        //Eigen::Vector3d v_body = q_wb.inverse() * v_ground_world; /// air velocity in body frame (assuming no wind)

        // 1D residual: body x velocity minus measured velocity 
        residuals[0] = (vx_meas - vx_pred) * sqrt_info;

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

    double vx_meas;    // measured velocity (m)
    double wind // get this int he correct frame
    double sqrt_info; // 1 / sigma