#ifndef UTILITY_H
#define UTILITY_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>                 // pcl::PointCloud
#include <pcl/point_types.h>                 // pcl::PointXYZ
#include <pcl_conversions/pcl_conversions.h> // pcl::fromROSMsg
#include <pcl/registration/ndt.h>            // NDT算法
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <string>

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

    using imu_ptr = std::shared_ptr<slam_czc::IMU>;

    imu_ptr convertToIMU(const sensor_msgs::Imu::ConstPtr &imu_msg);

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
        std::deque<imu_ptr> imu_;
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

    Eigen::Matrix3d wedge(const Eigen::Vector3d &v);
    Matrix3d rightJacobian(const Vector3d &phi);
    Eigen::Matrix3d inverseRightJacobian(const Vector3d &phi);
    Eigen::Matrix3d vectorToR(const Vector3d &theta);

    class State
    {
    public:
        State(double time) : timestamp_(time) {}
        State(double time, Eigen::Quaterniond q, Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d bg, Eigen::Vector3d bg) : timestamp_(time), q_(q), p_(p), v_(v), bg_(bg), ba_(ba) {}

        Eigen::Matrix4f getTransform();
        double timestamp_ = 0;
        Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d g_(0, 0, 9.81);
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
    inline Eigen::Matrix<S, 3, 1> vecFromArray(const std::vector<S> &v);

    template <typename S>
    inline Eigen::Matrix<S, 3, 3> matFromArray(const std::vector<S> &v);

}

#endif