#pragma once
// c++
#include <iostream>
#include <string>
#include <tr1/unordered_map>

// ros
#include <sensor_msgs/PointCloud2.h>

// eigen 
#include <Eigen/Core>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// cloud processing
#include "cloudProcessing.h"

#include "cloudMap.h"

#define THETA_THRESHOLD 0.0001
#define MIN_INI_COUNT 10
#define MIN_INI_TIME 3.0
#define MAX_GYR_VAR 0.5
#define MAX_ACC_VAR 0.6

extern bool debug_output;
extern std::string output_path;

extern bool time_diff_enable;
extern double time_diff;

extern float mov_threshold;

extern bool initial_flag;

extern Eigen::Vector3d G;
extern double G_norm;

bool time_list(point3D &point_1, point3D &point_2);

bool time_list_velodyne(velodyne_ros::Point &point_1, velodyne_ros::Point &point_2);

bool time_list_robosense(robosense_ros::Point &point_1, robosense_ros::Point &point_2);

bool time_list_ouster(ouster_ros::Point &point_1, ouster_ros::Point &point_2);

void point3DtoPCL(std::vector<point3D> &v_point_temp, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &p_cloud_temp);

Eigen::Matrix3d mat33FromArray(std::vector<double> &array);

Eigen::Vector3d vec3FromArray(std::vector<double> &array);

void pointBodyToWorld(pcl::PointXYZINormal const * const pi, pcl::PointXYZINormal * const po, Eigen::Matrix3d &R_world_cur, 
    Eigen::Vector3d &t_world_cur, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

void RGBpointLidarToIMU(pcl::PointXYZINormal const * const pi, pcl::PointXYZINormal * const po, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

bool planeFitting(Eigen::Matrix<double, 4, 1> &plane_parameter, const std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> &point, const double &threshold);

double AngularDistance(const Eigen::Matrix3d &rota, const Eigen::Matrix3d &rotb);

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b);

double AngularDistance(const Eigen::Vector3d &d_so3);

float calculateDist2(pcl::PointXYZINormal point1, pcl::PointXYZINormal point2);

void saveCutCloud(std::string &str, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &p_cloud_temp);

enum SizeParameterization
{
    NUM_MATCH_POINTS = 20
};

enum MotionCompensation
{
    IMU = 0,
    CONSTANT_VELOCITY = 1
};

enum StateInitialization
{
    INIT_IMU = 0,
    INIT_CONSTANT_VELOCITY = 1
};

class numType
{
  public:
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> normalizeR(const Eigen::MatrixBase<Derived> &R_in)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> q(R_in);
        q.normalize();
        return q.toRotationMatrix();
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &mat)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> mat_skew;
        mat_skew << typename Derived::Scalar(0), -mat(2), mat(1),
            mat(2), typename Derived::Scalar(0), -mat(0),
            -mat(1), mat(0), typename Derived::Scalar(0);
        return mat_skew;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 2> derivativeS2(const Eigen::MatrixBase<Derived> &g_in)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Matrix<Scalar_t, 3, 2> B_x;

        Eigen::Matrix<Scalar_t, 3, 1> g = g_in;

        g.normalize();

        B_x(0, 0) = static_cast<Scalar_t>(1.0) - g(0) * g(0) / (static_cast<Scalar_t>(1.0) + g(2));
        B_x(0, 1) = - g(0) * g(1) / (static_cast<Scalar_t>(1.0) + g(2));
        B_x(1, 0) = B_x(0, 1);
        B_x(1, 1) = static_cast<Scalar_t>(1.0) - g(1) * g(1) / (static_cast<Scalar_t>(1.0) + g(2));
        B_x(2, 0) = - g(0);
        B_x(2, 1) = - g(1);

        return B_x;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> rotFromV1toV2(const Eigen::MatrixBase<Derived> &v1_in, const Eigen::MatrixBase<Derived> &v2_in)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Matrix<Scalar_t, 3, 1> v1 = v1_in;
        Eigen::Matrix<Scalar_t, 3, 1> v2 = v2_in;

        v1.normalize();
        v2.normalize();

        Eigen::Matrix<Scalar_t, 3, 1> cross = v1.cross(v2);
        Scalar_t dot = v1.dot(v2);

        if (fabs(static_cast<Scalar_t>(1.0) - dot) < static_cast<Scalar_t>(1e-6))
            return Eigen::Matrix<Scalar_t, 3, 3>::Identity();

        Eigen::Matrix<Scalar_t, 3, 3> skew = skewSymmetric(cross);

        Eigen::Matrix<Scalar_t, 3, 3> R = Eigen::Matrix<Scalar_t, 3, 3>::Identity() + skew + skew * skew * (static_cast<Scalar_t>(1.0) - dot) 
            / (cross(0) * cross(0) + cross(1) * cross(1) + cross(2) * cross(2));

        return R;
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        return q;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> rotationToSo3(const Eigen::MatrixBase<Derived> &R_in)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Matrix<Scalar_t, 3, 3> R = normalizeR(R_in);
        Scalar_t theta = acos((R(0, 0) + R(1, 1) + R(2, 2) - static_cast<Scalar_t>(1.0)) / static_cast<Scalar_t>(2));

        if (theta < static_cast<Scalar_t>(THETA_THRESHOLD))
            return Eigen::Matrix<Scalar_t, 3, 1>(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) / static_cast<Scalar_t>(2.0);
        else
            return theta * Eigen::Matrix<Scalar_t, 3, 1>(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) / (static_cast<Scalar_t>(2.0) * sin(theta));
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> so3ToRotation(const Eigen::MatrixBase<Derived> &so3_in)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t theta = so3_in.norm();

        if (theta < static_cast<Scalar_t>(THETA_THRESHOLD))
        {
            Eigen::Matrix<Scalar_t, 3, 3> u_x = skewSymmetric(so3_in);
            return Eigen::Matrix<Scalar_t, 3, 3>::Identity() + u_x + static_cast<Scalar_t>(0.5) * u_x * u_x; 
        }
        else
        {
            Eigen::Matrix3d u_x = skewSymmetric(so3_in.normalized());
            return Eigen::Matrix<Scalar_t, 3, 3>::Identity() + sin(theta) * u_x + (static_cast<Scalar_t>(1) - cos(theta))* u_x * u_x;
        }
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> so3ToQuat(const Eigen::MatrixBase<Derived> &so3_in)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t theta = so3_in.norm();

        if (theta < static_cast<Scalar_t>(THETA_THRESHOLD))
        {
            Eigen::Matrix<Scalar_t, 3, 1> half_so3 = so3_in;
            half_so3 /= static_cast<Scalar_t>(2.0);

            Eigen::Quaternion<Scalar_t> q(static_cast<Scalar_t>(1.0), half_so3.x(), half_so3.y(), half_so3.z());
            q.normalize();
            return q;
        }
        else
        {
            Eigen::Matrix<Scalar_t, 3, 1> u = so3_in.normalized();

            Eigen::Quaternion<Scalar_t> q(cos(static_cast<Scalar_t>(0.5) * theta), u.x() * sin(static_cast<Scalar_t>(0.5) * theta), 
                u.y() * sin(static_cast<Scalar_t>(0.5) * theta), u.z() * sin(static_cast<Scalar_t>(0.5) * theta));
            q.normalize();
            return q;
        }
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> quatToSo3(const Eigen::QuaternionBase<Derived> &q_in)
    {
        return rotationToSo3(q_in.toRotationMatrix());
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> JleftSo3(const Eigen::MatrixBase<Derived> &so3_in)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t theta = so3_in.norm();

        if(theta < static_cast<Scalar_t>(THETA_THRESHOLD)){
            return Eigen::Matrix<Scalar_t, 3, 3>::Identity() + skewSymmetric(so3_in) / static_cast<Scalar_t>(2); 
        }
        else
        {
            Eigen::Matrix<Scalar_t, 3, 1> u = so3_in.normalized();
            return sin(theta) / theta * Eigen::Matrix<Scalar_t, 3, 3>::Identity() + (static_cast<Scalar_t>(1) - sin(theta) / theta) * u * u.transpose() + (static_cast<Scalar_t>(1) - cos(theta)) / theta * skewSymmetric(u); 
        }  
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> invJleftSo3(const Eigen::MatrixBase<Derived> &so3_in)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t theta = so3_in.norm();

        if(theta < static_cast<Scalar_t>(THETA_THRESHOLD))
        {
            return cos(theta / static_cast<Scalar_t>(2)) * Eigen::Matrix<Scalar_t, 3, 3>::Identity() 
                + static_cast<Scalar_t>(0.125) * so3_in * so3_in.transpose() - static_cast<Scalar_t>(0.5) * skewSymmetric(so3_in); 
        }
        else
        {
            Eigen::Matrix<Scalar_t, 3, 1> u = so3_in.normalized();
            return static_cast<Scalar_t>(0.5) * theta / tan(theta / static_cast<Scalar_t>(2)) * Eigen::Matrix<Scalar_t, 3, 3>::Identity() + 
                (static_cast<Scalar_t>(1) - static_cast<Scalar_t>(0.5) * theta / tan(theta / static_cast<Scalar_t>(2))) * u * u.transpose() - static_cast<Scalar_t>(0.5) * skewSymmetric(so3_in); 
        } 
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> JrightSo3(const Eigen::MatrixBase<Derived> &so3_in)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t theta = so3_in.norm();

        if (theta < static_cast<Scalar_t>(THETA_THRESHOLD))
            return Eigen::Matrix<Scalar_t, 3, 3>::Identity() - static_cast<Scalar_t>(0.5) * skewSymmetric(so3_in); 
        else
        {
            Eigen::Matrix<Scalar_t, 3, 1> u = so3_in.normalized();
            return sin(theta) / theta * Eigen::Matrix<Scalar_t, 3, 3>::Identity() + (static_cast<Scalar_t>(1) - sin(theta) / theta) * u * u.transpose() 
                - (static_cast<Scalar_t>(1) - cos(theta)) / theta * skewSymmetric(u); 
        } 
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> invJrightSo3(const Eigen::MatrixBase<Derived> &so3_in)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t theta = so3_in.norm();

        if(theta < static_cast<Scalar_t>(THETA_THRESHOLD))
            return cos(theta / static_cast<Scalar_t>(2)) * Eigen::Matrix<Scalar_t, 3, 3>::Identity() + static_cast<Scalar_t>(0.125) * so3_in * so3_in.transpose() +  static_cast<Scalar_t>(0.5) * skewSymmetric(so3_in); 
        else
        {
            Eigen::Matrix<Scalar_t, 3, 1> u = so3_in.normalized();
            return static_cast<Scalar_t>(0.5) * theta / tan(theta / static_cast<Scalar_t>(2)) * Eigen::Matrix3d::Identity() 
                + (static_cast<Scalar_t>(1) - static_cast<Scalar_t>(0.5) * theta / tan(theta / static_cast<Scalar_t>(2))) * u * u.transpose() + static_cast<Scalar_t>(0.5) * skewSymmetric(so3_in); 
        } 
    }
};

void subSampleFrame(std::vector<point3D> &frame, double size_voxel);

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling);

void distortFrameByConstant(std::vector<point3D> &points, std::vector<imuState> &imu_states, double time_frame_begin, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

void distortFrameByImu(std::vector<point3D> &points, std::vector<imuState> &imu_states, double time_frame_begin, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

void transformPoint(point3D &point_temp, Eigen::Quaterniond &q_end, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

void transformAllImuPoint(std::vector<point3D> &points, std::vector<imuState> &imu_states, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

namespace std {
    template <typename T, typename... Args>
        std::unique_ptr<T> make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}