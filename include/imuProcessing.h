#pragma once
// c++
#include <iostream>

// eigen 
#include <Eigen/Core>

// ceres
#include <ceres/ceres.h>

// utility
#include "utility.h"

class imuIntegration;

class state
{
public:

	Eigen::Quaterniond rotation;
	Eigen::Vector3d translation;
	Eigen::Vector3d velocity;
	Eigen::Vector3d ba;
	Eigen::Vector3d bg;
	imuIntegration *pre_integration;

	Eigen::Quaterniond rotation_begin;
	Eigen::Vector3d translation_begin;
	Eigen::Vector3d velocity_begin;
	Eigen::Vector3d ba_begin;
	Eigen::Vector3d bg_begin;

	std::vector<double> dt_buf;
	std::vector<Eigen::Quaterniond> rot_buf;
	std::vector<Eigen::Vector3d> trans_buf;
	std::vector<Eigen::Vector3d> velo_buf;
	std::vector<Eigen::Vector3d> un_acc_buf;
	std::vector<Eigen::Vector3d> un_omega_buf;

	state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, 
		const Eigen::Vector3d &velocity_, const Eigen::Vector3d& ba_, const Eigen::Vector3d& bg_);

	state(const state* state_temp, bool copy = false);

	void release();
};

class imuIntegration
{
public:

	double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    Eigen::Vector3d linearized_ba, linearized_bg;

    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 15, 15> step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Eigen::Matrix<double, 18, 18> noise;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;
    Eigen::Vector3d delta_g;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;

    double acc_cov;
	double gyr_cov;
	double b_acc_cov;
	double b_gyr_cov;

	const Eigen::Vector3d linearized_acc, linearized_gyr;

	imuIntegration() = delete;
	
    imuIntegration(const Eigen::Vector3d &acc_0_, const Eigen::Vector3d &gyr_0_,
                   const Eigen::Vector3d &linearized_ba_, const Eigen::Vector3d &linearized_bg_, 
                   const double acc_cov_, const double gyr_cov_, const double b_acc_cov_, const double b_gyr_cov_);

    imuIntegration(const imuIntegration* integration_temp, const Eigen::Vector3d &linearized_acc_, const Eigen::Vector3d &linearized_gyr_);

	void midPointIntegration(double dt_, 
                            const Eigen::Vector3d &acc_0_, const Eigen::Vector3d &gyr_0_,
                            const Eigen::Vector3d &acc_1_, const Eigen::Vector3d &gyr_1_,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian);

    void propagate(double dt_, const Eigen::Vector3d &acc_1_, const Eigen::Vector3d &gyr_1_);

    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);

    void repropagate(const Eigen::Vector3d &linearized_ba_, const Eigen::Vector3d &linearized_bg_);

    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &p_last, const Eigen::Quaterniond &q_last, const Eigen::Vector3d &v_last, const Eigen::Vector3d &ba_last, 
	const Eigen::Vector3d &bg_last, const Eigen::Vector3d &p_cur, const Eigen::Quaterniond &q_cur, const Eigen::Vector3d &v_cur, const Eigen::Vector3d &ba_cur, const Eigen::Vector3d &bg_cur);

	void release();
};

class imuProcessing
{
private:

	double acc_cov;
	double gyr_cov;
	double b_acc_cov;
	double b_gyr_cov;

	Eigen::Matrix3d R_imu_lidar;
	Eigen::Vector3d t_imu_lidar;

	bool first_imu;

	Eigen::Vector3d acc_0, gyr_0;

public:

	state *current_state;
	state *last_state;

	imuProcessing();

	// IMU parameter
	void setAccCov(double para);
	void setGyrCov(double para);
	void setBiasAccCov(double para);
	void setBiasGyrCov(double para);

	// extrinsic parameter
	void setExtrinR(Eigen::Matrix3d &R);
	void setExtrinT(Eigen::Vector3d &t);

	// function
	void process(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity, double timestamp);
};