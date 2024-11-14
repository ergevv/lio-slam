#ifndef UTILITY_H
#define UTILITY_H

#define PCL_NO_PRECOMPILE

#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <pcl/point_cloud.h>                 // pcl::PointCloud
#include <pcl/point_types.h>                 // pcl::PointXYZ
#include <pcl_conversions/pcl_conversions.h> // pcl::fromROSMsg
#include <pcl/registration/ndt.h>            // NDT算法
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include "pcl/impl/pcl_base.hpp"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/search/impl/organized.hpp"
#include "pcl/surface/impl/convex_hull.hpp"
// #include <pcl/filters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/impl/search.hpp>

// Ceres 头文件
#include <ceres/ceres.h>

namespace slam_czc
{

    /// IMU 读数
    struct IMU
    {
        IMU() = default;
        IMU(double t, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

        double timestamp_ = 0.0;
        Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d acce_ = Eigen::Vector3d::Zero();
    };

    using ImuPtr = std::shared_ptr<slam_czc::IMU>;

    ImuPtr convertToIMU(const sensor_msgs::Imu::ConstPtr &imu_msg);

    // 自定义点云结构
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        std::uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

//(type, member_name, field_name)。这里的 type 是字段的数据类型，member_name 是结构体成员变量的名字，field_name 是点云字段的名字。
POINT_CLOUD_REGISTER_POINT_STRUCT(slam_czc::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))

namespace slam_czc
{
    using PointType = pcl::PointCloud<slam_czc::Point>;
    // ::Ptr , pcl自带的h智能指针简易写法
    PointType::Ptr convertToPCL(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

    struct SortedData
    {
        double pc_begin_time_ = 0; // 仿真不考虑运动畸变，故没用
        double pc_end_time_ = 0;
        std::deque<ImuPtr> imu_;
        PointType::Ptr pc_ = nullptr;
    };

    template <typename A, typename B, typename GetData>
    void computeMeanAndCov(const A &data, B &mean, B &cov, GetData &&getter)
    {
        size_t len = data.size();
        assert(len > 1);
        mean = std::accumulate(data.begin(), data.end(), B::Zero().eval(), [&getter](const B &sum, const auto &data) -> B
                               { return sum + getter(data); }) /
               len;

        cov = std::accumulate(data.begin(), data.end(), B::Zero().eval(),
                              [&mean, &getter](const B &sum, const auto &data) -> B
                              {
                                  return sum + (getter(data) - mean).cwiseAbs2().eval();
                              }) /
              (len - 1);
    }

    class State
    {
    public:
        explicit State() {}
        explicit State(double time) : timestamp_(time) {}
        explicit State(double time, Eigen::Quaterniond q, Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d bg, Eigen::Vector3d ba) : timestamp_(time), q_(q), p_(p), v_(v), bg_(bg), ba_(ba) {}

        Eigen::Matrix4f getTransform();
        double timestamp_ = 0;
        Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d g_{0, 0, 9.81};
    };

    class Pose
    {
    public:
        Eigen::Vector3d p_;
        Eigen::Quaterniond q_;
        Eigen::Matrix4f transform;
        Pose() = default;
        Pose(Eigen::Vector3d p, Eigen::Quaterniond q) : p_(p), q_(q) {}
    };

    template <typename S>
    inline Eigen::Matrix<S, 3, 1> vecFromArray(const std::vector<S> &v)
    {
        return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
    }

    template <typename S>
    inline Eigen::Matrix<S, 3, 3> matFromArray(const std::vector<S> &v)
    {
        Eigen::Matrix<S, 3, 3> m;
        m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
        return m;
    }
    const double eps = 1e-8;
    template <typename Derived>
    Eigen::Quaternion<typename Derived::Scalar> theta2Q(const Eigen::MatrixBase<Derived> &theta);

    Eigen::Vector3d Q2theta(const Eigen::Quaterniond &q);

    Eigen::SparseMatrix<double> CRSMatrix2EigenMatrix(const ceres::CRSMatrix *crs);

    bool marginalize(const Eigen::MatrixXd &H, const Eigen::VectorXd &b, Eigen::MatrixXd &Jr, Eigen::VectorXd &br, const int m);
}

#endif