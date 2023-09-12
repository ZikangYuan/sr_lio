#pragma once
// c++
#include <iostream>

// eigen 
#include <Eigen/Core>

// ceres
#include <ceres/ceres.h>

// utility
#include "utility.h"

class state
{
public:

	Eigen::Quaterniond rotation;
	Eigen::Vector3d translation;
	Eigen::Vector3d velocity;
	Eigen::Vector3d ba;
	Eigen::Vector3d bg;

	state();

	state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, 
		const Eigen::Vector3d &velocity_, const Eigen::Vector3d& ba_, const Eigen::Vector3d& bg_);

	state(const state* state_temp, bool copy = false);

	void release();
};