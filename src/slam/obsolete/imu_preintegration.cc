#include "imu_preintegration.h"

namespace slam_czc
{

    IMUPreintegration::IMUPreintegration(Eigen::Vector3d bg, Eigen::Vector3d ba, double noise_gyro, double noise_acce) : bg_(bg), ba_(ba_)
    {
        noise_measure_.diagonal() << noise_gyro, noise_gyro, noise_gyro, noise_acce, noise_acce, noise_acce;
    }

    void IMUPreintegration::integrate(const IMU &imu, double dt)
    {
        // 去除零偏
        Eigen::Vector3d gyr = imu.gyro_ - bg_;
        Eigen::Vector3d acc = imu.acce_ - ba_;

        // 计算旋转、平移、u速度噪声协方差矩阵，用做优化的信息矩阵，评估误差的置信度
        Eigen::Matrix<double, 9, 9> A;
        A.setIdentity();
        Eigen::Matrix<double, 9, 6> B;
        B.setZero();

        Eigen::Matrix3d acc_hat = wedge(acc); // 向量-》反对称矩阵
        double dt2 = dt * dt;
        Eigen::Vector3d omega = gyr * dt;    // 转动量
        Mat3d rightJ = rightJacobian(omega); // 右雅可比
        Eigen::Quaterniond deltaq = Eigen::Quaterniond(1, gyr(0) * dt / 2, gyr(1) * dt / 2, gyr(3) * dt / 2);
        deltaq.normalized();
        Eigen::Matrix3d deltaR = deltaq.toRotationMatrix();

        Eigen::Matrix3d dR = dq_.toRotationMatrix();
        // NOTE A, B左上角块与公式稍有不同
        A.block<3, 3>(0, 0) = deltaR.transpose();
        A.block<3, 3>(3, 0) = -dR * dt * acc_hat;
        A.block<3, 3>(6, 0) = -0.5f * dR * acc_hat * dt2;
        A.block<3, 3>(6, 3) = dt * Eigen::Matrix3d::Identity();

        B.block<3, 3>(0, 0) = rightJ * dt;
        B.block<3, 3>(3, 3) = dR * dt;
        B.block<3, 3>(6, 3) = 0.5f * dR * dt2;

        // 更新噪声项的协方差
        cov_ = A * cov_ * A.transpose() + B * noise_measure_ * B.transpose();

        // 预积分量对零偏的导数
        dP_dba_ = dP_dba_ + dV_dba_ * dt - 0.5f * dR * dt2;
        dP_dbg_ = dP_dbg_ + dV_dbg_ * dt - 0.5f * dR * dt2 * acc_hat * dR_dbg_;
        dV_dba_ = dV_dba_ - dR * dt;
        dV_dbg_ = dV_dbg_ - dR * dt * acc_hat * dR_dbg_;
        dR_dbg_ = deltaR.transpose() * dR_dbg_ - rightJ * dt;

        // 更新预积分量,result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
        dq_ = dq_ * Eigen::Quaterniond(1, gyr(0) * dt / 2, gyr(1) * dt / 2, gyr(3) * dt / 2);
        dq_ = dq_.normalized();
        dp_ = dp_ + dv_ * dt + 0.5f * dq_.toRotationMatrix() * acc * dt * dt;
        dv_ = dv_ + dq_.toRotationMatrix() * acc * dt;
        dt_ += dt;
    }

    State IMUPreintegration::predict(State start, Eigen::Vector3d g)
    {
        Eigen::Quaterniond qj = start.q_ * dq_;
        Eigen::Vector3d vj = start.q_.toRotationMatrix() * dv_ + start.v_ + g * dt_;
        Eigen::Vector3d pj = start.q_.toRotationMatrix() * dp_ + start.p_ + start.v_ * dt_ + 0.5f * g * dt_ * dt_;

        auto state = State(start.timestamp_ + dt_, qj, pj, vj, bg_, ba_);

        return state;
    }

    Eigen::Quaterniond IMUPreintegration::getDeltaRotation(const Vec3d &bg) { return dq_ * (dR_dbg_ * (bg - bg_)).toQuaternion(); }

    Eigen::Vector3d IMUPreintegration::getDeltaVelocity(const Eigen::Vector3d &bg, const Eigen::Vector3d &ba)
    {
        return dv_ + dV_dbg_ * (bg - bg_) + dV_dba_ * (ba - ba_);
    }

    Eigen::Vector3d IMUPreintegration::getDeltaPosition(const Eigen::Vector3d &bg, const Eigen::Vector3d &ba)
    {
        return dp_ + dP_dbg_ * (bg - bg_) + dP_dba_ * (ba - ba_);
    }
}