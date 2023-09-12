// c++
#include <iostream>
#include <math.h>
#include <vector>

// eigen 
#include <Eigen/Core>

#include "cloudMap.h"
#include "lioOptimization.h"

#include <thread>

// utility
#include "utility.h"
#include "parameters.h"

optimizeSummary lioOptimization::buildPlaneResiduals(const icpOptions &cur_icp_options, const voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, 
    std::vector<planeParam> &plane_residuals, cloudFrame *p_frame, double &loss_sum)
{
    const short nb_voxels_visited = p_frame->frame_id < cur_icp_options.init_num_frames ? 2 : cur_icp_options.voxel_neighborhood;
    const int kMinNumNeighbors = cur_icp_options.min_number_neighbors;
    const int kThresholdCapacity = p_frame->frame_id < cur_icp_options.init_num_frames ? 1 : cur_icp_options.threshold_voxel_occupancy;

    state *last_state = all_cloud_frame[p_frame->id - sweep_cut_num]->p_state;
    state *current_state = p_frame->p_state;
    Eigen::Quaterniond end_quat = Eigen::Quaterniond(current_state->rotation);
    Eigen::Vector3d end_t = current_state->translation;

    auto transformKeypoints = [&]()
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        for (auto &keypoint: keypoints) {
            R = end_quat.normalized().toRotationMatrix();
            t = end_t;

            keypoint.point = R * (R_imu_lidar * keypoint.raw_point + t_imu_lidar) + t;
        }
    };

    auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
                                           Eigen::Vector3d &location, double &planarity_weight)
    {

        auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
        planarity_weight = std::pow(neighborhood.a2D, cur_icp_options.power_planarity);

        if (neighborhood.normal.dot(last_state->translation - location) < 0) {
            neighborhood.normal = -1.0 * neighborhood.normal;
        }
        return neighborhood;
    };

    double lambda_weight = std::abs(cur_icp_options.weight_alpha);
    double lambda_neighborhood = std::abs(cur_icp_options.weight_neighborhood);
    const double kMaxPointToPlane = cur_icp_options.max_dist_to_plane_icp;
    const double sum = lambda_weight + lambda_neighborhood;

    lambda_weight /= sum;
    lambda_neighborhood /= sum;

    int num_residuals = 0;
    int num_keypoints = keypoints.size();

    transformKeypoints();

    for (int k = 0; k < num_keypoints; k++) {
        auto &keypoint = keypoints[k];
        auto &raw_point = keypoint.raw_point;

        std::vector<voxel> voxels;
        auto vector_neighbors = searchNeighbors(voxel_map_temp, keypoint.point,
                                                 nb_voxels_visited, cur_icp_options.size_voxel_map,
                                                 cur_icp_options.max_number_neighbors, kThresholdCapacity,
                                                 cur_icp_options.estimate_normal_from_neighborhood ? nullptr : &voxels);

        if (vector_neighbors.size() < kMinNumNeighbors)
            continue;

        double weight;

        Eigen::Vector3d location = R_imu_lidar * raw_point + t_imu_lidar;

        auto neighborhood = estimatePointNeighborhood(vector_neighbors, location, weight);

        weight = lambda_weight * weight + lambda_neighborhood * std::exp(-(vector_neighbors[0] -
                 keypoint.point).norm() / (kMaxPointToPlane * kMinNumNeighbors));

        planeParam plane_temp;
        plane_temp.raw_point = location;
        plane_temp.norm_vector = neighborhood.normal;
        plane_temp.norm_vector.normalize();
        plane_temp.norm_offset = - plane_temp.norm_vector.dot(vector_neighbors[0]);
        plane_temp.distance = plane_temp.norm_vector.dot(end_quat.toRotationMatrix() * plane_temp.raw_point + end_t) + plane_temp.norm_offset;
        plane_temp.weight = weight;

        if (plane_temp.distance < cur_icp_options.max_dist_to_plane_icp) {
            num_residuals++;
            plane_temp.jacobians.block<1, 3>(0, 0) = plane_temp.norm_vector.transpose() * weight;
            plane_temp.jacobians.block<1, 3>(0, 3) = - plane_temp.norm_vector.transpose() * end_quat.toRotationMatrix() * numType::skewSymmetric(plane_temp.raw_point) * weight;
            plane_residuals.push_back(plane_temp);

            loss_sum += plane_temp.distance * plane_temp.distance;
        }

        if(num_residuals >= cur_icp_options.max_num_residuals) break;
    }

    if (num_residuals < cur_icp_options.min_number_neighbors)
    {
        std::stringstream ss_out;
        ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
        ss_out << "[Optimization] number_of_residuals : " << num_residuals << std::endl;
        optimizeSummary summary;
        summary.success = false;
        summary.num_residuals_used = num_residuals;
        summary.error_log = ss_out.str();
        if (cur_icp_options.debug_print) {
            std::cout << summary.error_log;
        }
        return summary;
    }
    else
    {
        optimizeSummary summary;
        summary.success = true;
        summary.num_residuals_used = num_residuals;
        return summary;
    }
}

optimizeSummary lioOptimization::updateIEKF(const icpOptions &cur_icp_options, const voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, cloudFrame *p_frame)
{    
    int max_num_iter = p_frame->frame_id < cur_icp_options.init_num_frames ? 
        std::max(15, cur_icp_options.num_iters_icp) : cur_icp_options.num_iters_icp;

    Eigen::Vector3d p_predict = eskf_pro->getTranslation();
    Eigen::Quaterniond q_predict = eskf_pro->getRotation();
    Eigen::Vector3d v_predict = eskf_pro->getVelocity();
    Eigen::Vector3d ba_predict = eskf_pro->getBa();
    Eigen::Vector3d bg_predict = eskf_pro->getBg();
    Eigen::Vector3d g_predict = eskf_pro->getGravity();

    optimizeSummary summary;

    for (int i = -1; i < max_num_iter; i++)
    {
        std::vector<planeParam> plane_residuals;

        double loss_old = 0.0;

        summary = buildPlaneResiduals(cur_icp_options, voxel_map_temp, keypoints, plane_residuals, p_frame, loss_old);

        if (summary.success == false)
            return summary;

        int num_plane_residuals = plane_residuals.size();

        Eigen::Matrix<double, Eigen::Dynamic, 6> H_x;
        Eigen::Matrix<double, Eigen::Dynamic, 1> h; 

        H_x.resize(num_plane_residuals, 6);
        h.resize(num_plane_residuals, 1);

        for (int i = 0; i < num_plane_residuals; i++)
        {
            H_x.block<1, 6>(i, 0) = plane_residuals[i].jacobians;
            h.block<1, 1>(i, 0) = Eigen::Matrix<double, 1, 1>(plane_residuals[i].distance * plane_residuals[i].weight);
        }

        Eigen::Vector3d d_p = eskf_pro->getTranslation() - p_predict;
        Eigen::Quaterniond d_q = q_predict.inverse() * eskf_pro->getRotation();
        Eigen::Vector3d d_so3 = numType::quatToSo3(d_q);
        Eigen::Vector3d d_v = eskf_pro->getVelocity() - v_predict;
        Eigen::Vector3d d_ba = eskf_pro->getBa() - ba_predict;
        Eigen::Vector3d d_bg = eskf_pro->getBg() - bg_predict;

        Eigen::Vector3d g = eskf_pro->getGravity();

        Eigen::Vector3d g_predict_normalize = g_predict;
        Eigen::Vector3d g_normalize = g;

        g_predict_normalize.normalize();
        g_normalize.normalize();

        Eigen::Vector3d cross = g_predict_normalize.cross(g_normalize);
        double dot = g_predict_normalize.dot(g_normalize);

        Eigen::Matrix3d R_dg;

        if (fabs(1.0 - dot) < 1e-6)
            R_dg = Eigen::Matrix3d::Identity();
        else
        {
            Eigen::Matrix3d skew = numType::skewSymmetric(cross);
            R_dg = Eigen::Matrix3d::Identity() + skew + skew * skew * (1.0 - dot) 
                / (cross(0) * cross(0) + cross(1) * cross(1) + cross(2) * cross(2));
        }

        Eigen::Vector3d so3_dg = numType::rotationToSo3(R_dg);
        Eigen::Matrix<double, 3, 2> B_x_predict = numType::derivativeS2(g_predict);
        Eigen::Vector2d d_g = B_x_predict.transpose() * so3_dg;

        Eigen::Matrix<double, 17, 1> d_x;
        d_x.head<3>() = d_p;
        d_x.segment<3>(3) = d_so3;
        d_x.segment<3>(6) = d_v;
        d_x.segment<3>(9) = d_ba;
        d_x.segment<3>(12) = d_bg;
        d_x.tail<2>() = d_g;

        Eigen::Matrix3d J_k_so3 = Eigen::Matrix3d::Identity() - 0.5 * numType::skewSymmetric(d_so3);
        Eigen::Matrix2d J_k_s2 = Eigen::Matrix2d::Identity() + 0.5 * B_x_predict.transpose() * numType::skewSymmetric(so3_dg) * B_x_predict;

        Eigen::Matrix<double, 17, 1> d_x_new = d_x;
        d_x_new.segment<3>(3) = J_k_so3 * d_so3;
        d_x_new.segment<2>(15) = J_k_s2 * d_g;

        Eigen::Matrix<double, 17, 17> covariance = eskf_pro->getCovariance();

        for (int j = 0; j < covariance.cols(); j++)
            covariance.block<3, 1>(3, j) = J_k_so3 * covariance.block<3, 1>(3, j);

        for (int j = 0; j < covariance.cols(); j++)
            covariance.block<2, 1>(15, j) = J_k_s2 * covariance.block<2, 1>(15, j);

        for (int j = 0; j < covariance.rows(); j++)
            covariance.block<1, 3>(j, 3) = covariance.block<1, 3>(j, 3) * J_k_so3.transpose();

        for (int j = 0; j < covariance.rows(); j++)
            covariance.block<1, 2>(j, 15) = covariance.block<1, 2>(j, 15) * J_k_s2.transpose();

        Eigen::Matrix<double, 17, 17> temp = (covariance/laser_point_cov).inverse();
        Eigen::Matrix<double, 6, 6> HTH = H_x.transpose() * H_x;
        temp.block<6, 6>(0, 0) += HTH;
        Eigen::Matrix<double, 17, 17> temp_inv = temp.inverse();

        Eigen::Matrix<double, 17, 1> K_h = temp_inv.block<17, 6>(0, 0) * H_x.transpose() * h;

        Eigen::Matrix<double, 17, 17> K_x = Eigen::Matrix<double, 17, 17>::Zero();
        K_x.block<17, 6>(0, 0) = temp_inv.block<17, 6>(0, 0) * HTH;

        d_x = - K_h + (K_x - Eigen::Matrix<double, 17, 17>::Identity()) * d_x_new;

        Eigen::Vector3d g_before = eskf_pro->getGravity();

        if ((d_x.head<3>()).norm() > 100.0 || AngularDistance(d_x.segment<3>(3)) > 100.0)
        {
            continue;
        }

        eskf_pro->observe(d_x);

        p_frame->p_state->translation = eskf_pro->getTranslation();
        p_frame->p_state->rotation = eskf_pro->getRotation();
        p_frame->p_state->velocity = eskf_pro->getVelocity();
        p_frame->p_state->ba = eskf_pro->getBa();
        p_frame->p_state->bg = eskf_pro->getBg();
        G = eskf_pro->getGravity();
        G_norm = G.norm();

        bool converage = false;

        if (p_frame->frame_id > sweep_cut_num && 
            (d_x.head<3>()).norm() < cur_icp_options.threshold_translation_norm && 
            AngularDistance(d_x.segment<3>(3)) < cur_icp_options.threshold_orientation_norm)
        {
            converage = true;
        }

        if (converage || i == max_num_iter - 1)
        {
            Eigen::Matrix<double, 17, 17> covariance_new = covariance;

            Eigen::Matrix<double, 3, 2> B_x_before = numType::derivativeS2(g_before);

            J_k_so3 = Eigen::Matrix3d::Identity() - 0.5 * numType::skewSymmetric(d_x.segment<3>(3));
            J_k_s2 = Eigen::Matrix2d::Identity() + 0.5 * B_x_before.transpose() * numType::skewSymmetric(B_x_before * d_x.tail<2>()) * B_x_before;

            for (int j = 0; j < covariance.cols(); j++)
                covariance_new.block<3, 1>(3, j) = J_k_so3 * covariance.block<3, 1>(3, j);

            for (int j = 0; j < covariance.cols(); j++)
                covariance_new.block<2, 1>(15, j) = J_k_s2 * covariance.block<2, 1>(15, j);

            for (int j = 0; j < covariance.rows(); j++)
            {
                covariance_new.block<1, 3>(j, 3) = covariance.block<1, 3>(j, 3) * J_k_so3.transpose();
                covariance.block<1, 3>(j, 3) = covariance.block<1, 3>(j, 3) * J_k_so3.transpose();
            }

            for (int j = 0; j < covariance.rows(); j++)
            {
                covariance_new.block<1, 2>(j, 15) = covariance.block<1, 2>(j, 15) * J_k_s2.transpose();
                covariance.block<1, 2>(j, 15) = covariance.block<1, 2>(j, 15) * J_k_s2.transpose();
            }

            for (int j = 0; j < 6; j++)
                K_x.block<3, 1>(3, j) = J_k_so3 * K_x.block<3, 1>(3, j);

            for (int j = 0; j < 6; j++)
                K_x.block<2, 1>(15, j) = J_k_s2 * K_x.block<2, 1>(15, j);

            covariance = covariance_new - K_x.block<17, 6>(0, 0) * covariance.block<6, 17>(0, 0);

            eskf_pro->setCovariance(covariance);

            break;
        }
    }

    return summary;
}

Neighborhood lioOptimization::computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
{
    Neighborhood neighborhood;
    // Compute the normals
    Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
    for (auto &point: points) {
        barycenter += point;
    }

    barycenter /= (double) points.size();
    neighborhood.center = barycenter;

    Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
    for (auto &point: points) {
        for (int k = 0; k < 3; ++k)
            for (int l = k; l < 3; ++l)
                covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                           (point(l) - barycenter(l));
    }
    covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
    covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
    covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
    neighborhood.covariance = covariance_Matrix;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
    Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
    neighborhood.normal = normal;

    double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
    double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
    double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
    neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

    if (neighborhood.a2D != neighborhood.a2D) {
        throw std::runtime_error("error");
    }

    return neighborhood;
}

using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

struct comparator {
    bool operator()(const pair_distance_t &left, const pair_distance_t &right) const {
        return std::get<0>(left) < std::get<0>(right);
    }
};

using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lioOptimization::searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
        int nb_voxels_visited, double size_voxel_map, int max_num_neighbors, int threshold_voxel_capacity, std::vector<voxel> *voxels)
{

    if (voxels != nullptr)
        voxels->reserve(max_num_neighbors);

    short kx = static_cast<short>(point[0] / size_voxel_map);
    short ky = static_cast<short>(point[1] / size_voxel_map);
    short kz = static_cast<short>(point[2] / size_voxel_map);

    priority_queue_t priority_queue;

    voxel voxel_temp(kx, ky, kz);
    for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
        for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
            for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
                voxel_temp.x = kxx;
                voxel_temp.y = kyy;
                voxel_temp.z = kzz;

                auto search = map.find(voxel_temp);
                if (search != map.end()) {
                    const auto &voxel_block = search.value();
                    if (voxel_block.NumPoints() < threshold_voxel_capacity)
                        continue;
                    for (int i(0); i < voxel_block.NumPoints(); ++i) {
                        auto &neighbor = voxel_block.points[i];
                        double distance = (neighbor - point).norm();
                        if (priority_queue.size() == max_num_neighbors) {
                            if (distance < std::get<0>(priority_queue.top())) {
                                priority_queue.pop();
                                priority_queue.emplace(distance, neighbor, voxel_temp);
                            }
                        } else
                            priority_queue.emplace(distance, neighbor, voxel_temp);
                    }
                }
            }
        }
    }

    auto size = priority_queue.size();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(size);
    if (voxels != nullptr) {
        voxels->resize(size);
    }
    for (auto i = 0; i < size; ++i) {
        closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
        if (voxels != nullptr)
            (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
        priority_queue.pop();
    }


    return closest_neighbors;
}

optimizeSummary lioOptimization::optimize(cloudFrame *p_frame, const icpOptions &cur_icp_options, double sample_voxel_size)
{
    std::vector<point3D> keypoints;
    gridSampling(p_frame->point_frame, keypoints, sample_voxel_size);

    optimizeSummary optimize_summary;

    optimize_summary = updateIEKF(cur_icp_options, voxel_map, keypoints, p_frame);

    if (!optimize_summary.success) {
        return optimize_summary;
    }

    Eigen::Quaterniond q_end = p_frame->p_state->rotation;
    Eigen::Vector3d t_end = p_frame->p_state->translation;
    for (auto &point_temp: p_frame->point_frame) {
        transformPoint(point_temp, q_end, t_end, R_imu_lidar, t_imu_lidar);
    }

    return optimize_summary;
}