#ifndef IMU_PREINTEGRATION_H
#define IMU_PREINTEGRATION_H

#include "utility.h"

namespace slam_czc
{
    class IMUPreintegration
    {
    public:
        IMUPreintegration(Eigen::Vector3d bg, Eigen::Vector3d ba, double noise_gyro, double noise_acce);
        void integrate(const IMU &imu, double dt);
        State predict(State start, Eigen::Vector3d g);

        Eigen::Quaterniond getDeltaRotation(const Vec3d &bg) {}

        Eigen::Vector3d getDeltaVelocity(const Eigen::Vector3d &bg, const Eigen::Vector3d &ba){}

        Eigen::Vector3d getDeltaPosition(const Eigen::Vector3d &bg, const Eigen::Vector3d &ba){}

        Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();
        Eigen::Matrix<double, 6, 6> noise_measure_ = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 9, 9> cov_ = Eigen::Matrix<double, 9, 9>::Zero();

        // 预积分量
        Eigen::Quaterniond dq_ = Eigen::Quaterniond::Identity();

        Eigen::Vector3d dv_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d dp_ = Eigen::Vector3d::Zero();

        // 对零偏的求导
        Eigen::Matrix3d dR_dbg_ = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d dV_dbg_ = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d dV_dba_ = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d dP_dbg_ = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d dP_dba_ = Eigen::Matrix3d::Zero();

        //
        double dt_ = 0; // 整体时间
    };
}

#endif