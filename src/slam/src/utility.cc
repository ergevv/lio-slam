#include "utility.h"

namespace slam_czc
{

    ImuPtr convertToIMU(const sensor_msgs::Imu::ConstPtr &imu_msg)
    {
        double timestamp = imu_msg->header.stamp.toSec();
        Eigen::Vector3d gyro(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        Eigen::Vector3d acce(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);

        return std::make_shared<IMU>(timestamp, gyro, acce);
    }

    PointType::Ptr convertToPCL(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
    {
        PointType::Ptr cloud(new PointType);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        return cloud;
    }

    Eigen::Matrix4f State::getTransform()
    {
        // 创建一个4x4的单位矩阵
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

        // 将旋转矩阵放入4x4变换矩阵的左上角
        transform.block<3, 3>(0, 0) = q_.toRotationMatrix();

        // 设置平移部分
        transform(0, 3) = p_.x();
        transform(1, 3) = p_.y();
        transform(2, 3) = p_.z();
        return transform.cast<float>();
    }



    template <typename Derived>
    Eigen::Quaternion<typename Derived::Scalar> theta2Q(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

}