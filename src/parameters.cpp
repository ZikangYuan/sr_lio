#include "parameters.h"

odometryOptions odometryOptions::defaultDrivingProfile() {
    return odometryOptions{};
}

odometryOptions odometryOptions::robustDrivingProfile()
{
    odometryOptions default_options;

    default_options.voxel_size = 0.5;
    default_options.sample_voxel_size = 1.5;
    default_options.max_distance = 200.0;
    default_options.min_distance_points = 0.15;
    default_options.init_num_frames = 20;
    default_options.num_for_initialization = 10;
    default_options.max_num_points_in_voxel = 20;
    default_options.min_distance_points = 0.05;
    default_options.distance_error_threshold = 5.0;
    default_options.motion_compensation = CONSTANT_VELOCITY;
    default_options.initialization = INIT_CONSTANT_VELOCITY;

    auto &optimize_options = default_options.optimize_options;
    optimize_options.debug_print = false;
    optimize_options.init_num_frames = 40;
    optimize_options.max_number_neighbors = 20;
    optimize_options.min_number_neighbors = 20;
    optimize_options.num_iters_icp = 15;
    optimize_options.max_dist_to_plane_icp = 0.5;
    optimize_options.threshold_orientation_norm = 0.1;
    optimize_options.threshold_orientation_norm = 0.01;
    optimize_options.num_closest_neighbors = 1;

    return default_options;
}

odometryOptions odometryOptions::defaultRobustOutdoorLowInertia()
{
    odometryOptions default_options;
    default_options.voxel_size = 0.3;
    default_options.sample_voxel_size = 1.5;
    default_options.min_distance_points = 0.1;
    default_options.max_distance = 200.0;
    default_options.init_num_frames = 20;
    default_options.num_for_initialization = 10;
    default_options.max_num_points_in_voxel = 20;
    default_options.distance_error_threshold = 5.0;
    default_options.motion_compensation = CONSTANT_VELOCITY;
    default_options.initialization = INIT_CONSTANT_VELOCITY;

    auto &optimize_options = default_options.optimize_options;
    optimize_options.size_voxel_map = 0.8;
    optimize_options.num_iters_icp = 30;
    optimize_options.threshold_voxel_occupancy = 5;
    optimize_options.min_number_neighbors = 20;
    optimize_options.voxel_neighborhood = 1;

    optimize_options.init_num_frames = 20;
    optimize_options.max_number_neighbors = 20;
    optimize_options.min_number_neighbors = 20;
    optimize_options.max_dist_to_plane_icp = 0.5;
    optimize_options.threshold_orientation_norm = 0.1;
    optimize_options.threshold_orientation_norm = 0.01;
    optimize_options.num_closest_neighbors = 1;
    optimize_options.weight_neighborhood = 0.2;
    optimize_options.weight_alpha = 0.8;
    optimize_options.max_num_residuals = 600;
    optimize_options.min_num_residuals = 200;

    return default_options;
}

void odometryOptions::recordParameters()
{
	std::string str_temp;

	std::ofstream foutC(std::string(output_path + "/parameter_list.txt"), std::ios::app);

	foutC << "init_voxel_size: " << init_voxel_size << std::endl;
	foutC << "init_sample_voxel_size: " << init_sample_voxel_size << std::endl;
	foutC << "init_num_frames: " << init_num_frames << std::endl;
	foutC << "num_for_initialization: " << num_for_initialization << std::endl;
	foutC << "voxel_size: " << voxel_size << std::endl;
	foutC << "sample_voxel_size: " << sample_voxel_size << std::endl;
	foutC << "max_distance: " << max_distance << std::endl;
	foutC << "max_num_points_in_voxel: " << max_num_points_in_voxel << std::endl;
	foutC << "min_distance_points: " << min_distance_points << std::endl;
	foutC << "distance_error_threshold: " << distance_error_threshold << std::endl;

	switch(motion_compensation)
	{
	case 0:
		str_temp = "IMU";
		break;
	case 1:
		str_temp = "CONSTANT_VELOCITY";
		break;
	default:
		break;
	}
	foutC << "motion_compensation: " << str_temp << std::endl;
	switch(initialization)
	{
	case 0:
		str_temp = "INIT_IMU";
		break;
	case 1:
		str_temp = "INIT_CONSTANT_VELOCITY";
		break;
	default:
		break;
	}
	foutC << "initialization: " << str_temp << std::endl;
    
    foutC.close();

    optimize_options.recordParameters();
}

void icpOptions::recordParameters()
{
	std::string str_temp;

	std::ofstream foutC(std::string(output_path + "/parameter_list.txt"), std::ios::app);

	foutC << "threshold_voxel_occupancy: " << threshold_voxel_occupancy << std::endl;
	foutC << "init_num_frames: " << init_num_frames << std::endl;
	foutC << "size_voxel_map: " << size_voxel_map << std::endl;
	foutC << "num_iters_icp: " << num_iters_icp << std::endl;
	foutC << "min_number_neighbors: " << min_number_neighbors << std::endl;
	foutC << "voxel_neighborhood: " << voxel_neighborhood << std::endl;
	foutC << "power_planarity: " << power_planarity << std::endl;
	str_temp = estimate_normal_from_neighborhood ? "true" : "false";
	foutC << "estimate_normal_from_neighborhood: " << str_temp << std::endl;
	foutC << "max_number_neighbors: " << max_number_neighbors << std::endl;
	foutC << "max_dist_to_plane_icp: " << max_dist_to_plane_icp << std::endl;
	foutC << "threshold_orientation_norm: " << threshold_orientation_norm << std::endl;
	foutC << "threshold_translation_norm: " << threshold_translation_norm << std::endl;
	foutC << "max_num_residuals: " << max_num_residuals << std::endl;
	foutC << "min_num_residuals: " << min_num_residuals << std::endl;
	foutC << "num_closest_neighbors: " << num_closest_neighbors << std::endl;
	foutC << "weight_alpha: " << weight_alpha << std::endl;
	foutC << "weight_neighborhood: " << weight_neighborhood << std::endl;
	
	str_temp = debug_print ? "true" : "false";
	foutC << "debug_print: " << str_temp << std::endl;
	str_temp = debug_viz ? "true" : "false";
	foutC << "debug_viz: " << str_temp << std::endl;

	foutC.close();
}