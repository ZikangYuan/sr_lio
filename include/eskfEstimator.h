#pragma once
// c++
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>

// eigen 
#include <Eigen/Core>

// ceres
#include <ceres/ceres.h>

// utility
#include "utility.h"

class eskfEstimator
{
private:
    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    Eigen::Vector3d acc_cov, gyr_cov;
    Eigen::Vector3d acc_cov_scale, gyr_cov_scale;
    Eigen::Vector3d b_acc_cov, b_gyr_cov;

    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    Eigen::Vector3d v;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;
    Eigen::Vector3d g;

    Eigen::Matrix<double, 12, 12> noise;
    Eigen::Matrix<double, 17, 1> delta_state;
    Eigen::Matrix<double, 17, 17> covariance;

    Eigen::Matrix<double, 3, 2> lxly;

    Eigen::Vector3d mean_gyr, mean_acc;

    bool is_first_imu_meas;
    double time_first_imu;
    int num_init_meas;

    void initialization(const std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> &imu_meas);

    void initializeNoise();

    void midPointIntegration(Eigen::Vector3d &result_p, Eigen::Quaterniond &result_q, Eigen::Vector3d &result_v);

    void updateAndReset();

    void projectCovariance();

public:
    eskfEstimator();

    void tryInit(const std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> &imu_meas);

    void setAccCov(double para);

    void setGyrCov(double para);

    void setBiasAccCov(double para);

    void setBiasGyrCov(double para);

    void initializeImuData(const Eigen::Vector3d &acc_0_, const Eigen::Vector3d &gyr_0_);

    void setTranslation(const Eigen::Vector3d &p_);

    void setRotation(const Eigen::Quaterniond &q_);

    void setVelocity(const Eigen::Vector3d &v_);

    void setBa(const Eigen::Vector3d &ba_);

    void setBg(const Eigen::Vector3d &bg_);

    void setGravity(const Eigen::Vector3d &g_);

    void setCovariance(const Eigen::Matrix<double, 17, 17> &covariance_);

    Eigen::Matrix<double, 17, 17> getCovariance();

    Eigen::Vector3d getTranslation();

    Eigen::Quaterniond getRotation();

    Eigen::Vector3d getVelocity();

    Eigen::Vector3d getBa();

    Eigen::Vector3d getBg();

    Eigen::Vector3d getGravity();

    Eigen::Vector3d getLastAcc();

    Eigen::Vector3d getLastGyr();

    void predict(double dt_, const Eigen::Vector3d &acc_1_, const Eigen::Vector3d &gyr_1_);

    void observe(const Eigen::Matrix<double, 17, 1> &d_x_);

    void observePose(const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation, double trans_noise = 0.001, double ang_noise = 0.001);

    void calculateLxly();
};