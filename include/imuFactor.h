#pragma once
// c++
#include <iostream>

// eigen 
#include <Eigen/Core>

// ceres
#include <ceres/ceres.h>

// utility
#include "utility.h"

#include "imuProcessing.h"

class ImuFactor : public ceres::SizedCostFunction<15, 3, 4, 9>
{
public:
    ImuFactor(imuIntegration* pre_integration_, state* last_state_);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    
    void check(double **parameters);

    imuIntegration* pre_integration;

    Eigen::Quaterniond rot_last;
    Eigen::Vector3d tran_last;
    Eigen::Vector3d velocity_last;
    Eigen::Vector3d ba_last;
    Eigen::Vector3d bg_last;
};

class CTImuFactor : public ceres::SizedCostFunction<15, 3, 4, 9, 3, 4, 9>
{
public:
    CTImuFactor(imuIntegration* pre_integration_, int beta_);

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    
    void check(double **parameters);

    imuIntegration* pre_integration;

    int beta;
};