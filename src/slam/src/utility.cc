#include "utility.h"

namespace slam_czc
{

    imu_ptr convertToIMU(const sensor_msgs::Imu::ConstPtr &imu_msg)
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

    Eigen::Matrix3d wedge(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d S;
        S << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
        return S;
    }

    // 计算右雅可比矩阵
    Eigen::Matrix3d rightJacobian(const Vector3d &phi)
    {
        double phi_norm = phi.norm();
        if (phi_norm < 1e-5)
        { // 如果接近零，则使用泰勒展开
            return Eigen::Matrix3d::Identity() + 0.5 * wedge(phi) - 1.0 / 6.0 * wedge(phi) * wedge(phi);
        }
        else
        {
            Eigen::Matrix3d Jr = Eigen::Matrix3d::Identity();
            Jr -= (1 - std::cos(phi_norm)) / (phi_norm * phi_norm) * wedge(phi);
            Jr += (phi_norm - std::sin(phi_norm)) / (phi_norm * phi_norm * phi_norm) * wedge(phi) * wedge(phi);
            return Jr;
        }
    }

    // 计算右雅可比矩阵的逆
    Eigen::Matrix3d inverseRightJacobian(const Vector3d &phi)
    {
        double phi_norm = phi.norm();
        if (phi_norm < 1e-5)
        { // 如果接近零，则使用泰勒展开
            return Eigen::Matrix3d::Identity() - 0.5 * wedge(phi) - 1.0 / 24.0 * wedge(phi) * wedge(phi);
        }
        else
        {
            Eigen::Matrix3d Jr_inv = Eigen::Matrix3d::Identity();
            Jr_inv += 0.5 * wedge(phi);
            Jr_inv += (1.0 / phi_norm / phi_norm - (1 + std::cos(phi_norm)) / (2 * phi_norm * std::sin(phi_norm))) * wedge(phi) * wedge(phi);
            return Jr_inv;
        }
    }

    // 函数：从旋转矢量到旋转矩阵
    Eigen::Matrix3d vectorToR(const Vector3d &theta)
    {
        Eigen::Matrix3d R;
        double theta_norm = theta.norm();

        if (theta_norm < 1e-7)
        {
            // 当旋转角度非常小时，可以使用泰勒展开近似
            R = Eigen::Matrix3d::Identity(3, 3) +
                wedge(theta) +
                0.5 * wedge(wedge(theta) * theta);
        }
        else
        {
            // 使用罗德里格斯公式计算旋转矩阵
            Eigen::Vector3d unit_theta = theta / theta_norm;
            double sin_theta = std::sin(theta_norm);
            double cos_theta = std::cos(theta_norm);

            R = Eigen::Matrix3d::Identity(3, 3) +
                sin_theta * wedge(unit_theta) +
                (1 - cos_theta) * wedge(unit_theta) * wedge(unit_theta);
        }

        return R;
    }

    Eigen::Matrix4f State::getTransform()
    {
        // 创建一个4x4的单位矩阵
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // 将旋转矩阵放入4x4变换矩阵的左上角
        transform.block<3, 3>(0, 0) = q_.toRotationMatrix();

        // 设置平移部分
        transform(0, 3) = p_.x();
        transform(1, 3) = p_.y();
        transform(2, 3) = p_.z();
        return transform;
    }

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

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> theta2Q(const Eigen::MatrixBase<Derived> &theta)
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