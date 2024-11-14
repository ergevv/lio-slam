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

    Eigen::Vector3d Q2theta(const Eigen::Quaterniond &q)
    {
        double w = q.w();
        double x = q.x();
        double y = q.y();
        double z = q.z();

        // 计算旋转角
        double theta = 2 * std::acos(w);

        // 防止除以零的情况
        if (theta < 1e-6)
        {
            return Eigen::Vector3d::Zero(); // 当接近零旋转时，返回零向量
        }

        // 计算旋转轴
        double s = std::sqrt(1 - w * w); // |q| 应该等于 1，即 w^2 + x^2 + y^2 + z^2 = 1
        double u_x = x / s;
        double u_y = y / s;
        double u_z = z / s;

        // 计算李代数 so(3) 向量
        return theta * Eigen::Vector3d(u_x, u_y, u_z);
    }

    // 辅助函数：将CRSMatrix转换为Eigen SparseMatrix
    Eigen::SparseMatrix<double> CRSMatrix2EigenMatrix(const ceres::CRSMatrix *crs)
    {
        Eigen::SparseMatrix<double> sparseMat(crs->num_rows, crs->num_cols);
        for (int i = 0; i < crs->num_rows; ++i)
        {
            for (int j = crs->rows[i]; j < crs->rows[i + 1]; ++j)
            {
                sparseMat.insert(i, crs->cols[j]) = crs->values[j];
            }
        }
        return sparseMat;
    }

    bool marginalize(const Eigen::MatrixXd &H, const Eigen::VectorXd &b, Eigen::MatrixXd &Jr, Eigen::VectorXd &Er, const int m)
    {
        //$$(H_{22} - H_{12}^TH_{11}^{-1}H_{12})\delta x_2 = b_2 - H_{12}^TH_{11}^{-1}b_1$$

        // Amm矩阵的构建是为了保证其正定性
        Eigen::MatrixXd Amm = 0.5 * (H.block(0, 0, m, m) + H.block(0, 0, m, m).transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm); // 特征值分解

        // ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());
        //  一个逆矩阵的特征值是原矩阵的倒数，特征向量相同　select类似c++中 ? :运算符
        //  利用特征值取逆来构造其逆矩阵
        Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();

        Eigen::VectorXd bmm = b.segment(0, m);     // 带边缘化的大小
        Eigen::MatrixXd Amr = H.block(0, m, m, m); // 对应的四块矩阵
        Eigen::MatrixXd Arm = H.block(m, 0, m, m);
        Eigen::MatrixXd Arr = H.block(m, m, m, m);
        Eigen::VectorXd brr = b.segment(m, m); // 剩下的参数
        Eigen::MatrixXd H2 = Arr - Arm * Amm_inv * Amr;
        Eigen::VectorXd b2 = brr - Arm * Amm_inv * bmm;

        // 通过海森矩阵、b 求解雅可比矩阵和残差
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H2);
        // 特征值取逆
        Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
        Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

        Eigen::VectorXd S_sqrt = S.cwiseSqrt();
        Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
        Jr = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();

        Er =  S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * (-b2);
        return true;
    }

}