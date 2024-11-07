#ifndef FACTOR_H
#define FACTOR_H
#include "utility.h"
#include "integration_base.h"
#include "ceres/ceres.h"

namespace slam_czc
{

    struct IMUFactor
    {
        IMUFactor(std::shared_ptr<IntegrationBase> imu_pre,
                  const Eigen::Vector3d g)
            : imu_pre_(imu_pre), g_(g), dt_(imu_pre->sum_dt) { sqrt_info_ = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(imu_pre_->covariance.inverse()).matrixL().transpose(); }

        template <typename T>
        // last_q_, last_p_, last_v_,  last_bg_, last_ba_,current_q_, current_p_, current_v_,current_bg_, current_ba_
        bool operator()(const T *last_q_, const T *last_p_, const T *last_v_, const T *last_bg_, const T *last_ba_, const T *current_q_, const T *current_p_, const T *current_v_, const T *current_bg_, const T *current_ba_, T *residuals) const
        {
            Eigen::Map<const Eigen::Quaternion<T>> q_last(last_q_);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_last(last_p_);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> v_last(last_v_);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> bg_last(last_bg_);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba_last(last_ba_);
            Eigen::Map<const Eigen::Quaternion<T>> q_current(current_q_);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_current(current_p_);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> v_current(current_v_);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> bg_current(current_bg_);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba_current(current_ba_);
            Eigen::Map<Eigen::Matrix<T, 15, 1>> residual(residuals);

            const Eigen::Matrix<T, 3, 1> delta_p = imu_pre_->getDeltaPosition(bg_current, ba_current);
            const Eigen::Matrix<T, 3, 1> delta_v = imu_pre_->getDeltaVelocity(bg_current, ba_current);
            const Eigen::Quaternion<T> delta_q = imu_pre_->getDeltaRotation(bg_current);
            const Eigen::Matrix<T, 3, 3> delta_R = delta_q.toRotationMatrix();

            // 计算旋转误差
            Eigen::Matrix<T, 3, 3> R_last = q_last.toRotationMatrix().transpose();
            Eigen::Matrix<T, 3, 3> eR = delta_R.transpose() * R_last * q_current.toRotationMatrix().transpose();
            Eigen::AngleAxis<T> aa(eR);
            Eigen::Matrix<T, 3, 1> er = aa.angle() * aa.axis();
            residual.head(3) = er;

            // 计算速度误差
            const Eigen::Matrix<T, 3, 1> ev = R_last * (v_current - v_last - g_.cast<T>() * dt_) - delta_v;
            residual.template block<3, 1>(3, 0) = ev;

            // 计算位置误差
            const Eigen::Matrix<T, 3, 1> ep = R_last * (p_current - p_last - v_last * dt_ - (g_ * dt_ * dt_ / 2).cast<T>()) - delta_p;
            residual.template block<3, 1>(6, 0) = ep;

            // 计算陀螺仪偏差误差
            residual.template block<3, 1>(9, 0) = bg_current - bg_last;

            // 计算加速度计偏差误差
            residual.template block<3, 1>(12, 0) = ba_current - ba_last;

            // 应用信息矩阵
            residual = sqrt_info_.cast<T>() * residual;

            return true;
        }

        static ceres::CostFunction *Create(const std::shared_ptr<IntegrationBase> imu_pre,
                                           const Eigen::Vector3d g)
        {
            return (new ceres::AutoDiffCostFunction<
                    IMUFactor, 15, 4, 3, 3, 3, 3, 4, 3, 3, 3, 3>( // 第一个参数是CostFunction，第二个参数是残差的维数，第三个参数是参数块的维数，上一次旋转矩阵4维，平移3维，速度3维，零偏两个3维，这次的旋转4维、平移、速度3维、零偏两个3维
                new IMUFactor(imu_pre, g)));
        }

        std::shared_ptr<IntegrationBase> imu_pre_ = nullptr;
        Eigen::Vector3d g_;
        double dt_ = 0;
        Eigen::Matrix<double, 15, 15> sqrt_info_;
    };

    struct MargFactor
    {
        MargFactor(const State &state, const Eigen::Matrix<double, 15, 15> &prior_info)
            : state_(state)
        {
            prior_info_ = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(prior_info.inverse()).matrixL().transpose();
        }

        template <typename T>
        bool operator()(const T *q, const T *p, const T *v, const T *bg, const T *ba, T *residuals) const
        {
            Eigen::Map<const Eigen::Quaternion<T>> q_last(q);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_last(p);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> v_last(v);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> bg_last(bg);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba_last(ba);
            Eigen::Map<Eigen::Matrix<T, 15, 1>> residual(residuals);

            // 计算旋转误差
            Eigen::Matrix<T, 3, 3> eR = state_.q_.inverse().toRotationMatrix().cast<T>() * q_last.toRotationMatrix();
            Eigen::AngleAxis<T> aa(eR);
            Eigen::Matrix<T, 3, 1> er = aa.angle() * aa.axis();

            // 设置残差的前3个元素
            residual.head(3) = er;

            // 计算位置误差
            residual.template block<3, 1>(3, 0) = p_last - state_.p_.cast<T>();

            // 计算速度误差
            residual.template block<3, 1>(6, 0) = v_last - state_.v_.cast<T>();

            // 计算陀螺仪偏差误差
            residual.template block<3, 1>(9, 0) = bg_last - state_.bg_.cast<T>();

            // 计算加速度计偏差误差
            residual.template block<3, 1>(12, 0) = ba_last - state_.ba_.cast<T>();

            // 应用信息矩阵
            residual = prior_info_.cast<T>() * residual;
            return true;
        }

        static ceres::CostFunction *
        Create(const State &state, const Eigen::Matrix<double, 15, 15> &prior_info)
        {
            return (new ceres::AutoDiffCostFunction<
                    MargFactor, 15, 4, 3, 3, 3, 3>(new MargFactor(state, prior_info)));
        }

        State state_;
        Eigen::Matrix<double, 15, 15> prior_info_;
    };

    struct PointCloudFactor
    {
        PointCloudFactor(const Eigen::Matrix4d &pc_pose, const Eigen::Matrix<double, 6, 6> &ndt_info)
        {
            ndt_info_ = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(ndt_info.inverse()).matrixL().transpose();
            // 提取平移部分
            ndt_p_ = pc_pose.block<3, 1>(0, 3);

            // 提取旋转部分为旋转矩阵
            Eigen::Matrix3d rotation_matrix = pc_pose.block<3, 3>(0, 0);

            // 将旋转矩阵转换为四元数
            ndt_q_ = Eigen::Quaterniond(rotation_matrix);
        }

        template <typename T>
        bool operator()(const T *q, const T *p, T *residuals) const
        {
            Eigen::Map<const Eigen::Quaternion<T>> q_current(q);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_current(p);
            Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residuals);

            // 计算旋转误差
            Eigen::Matrix<T, 3, 3> eR = ndt_q_.inverse().toRotationMatrix().cast<T>() * q_current.toRotationMatrix();
            Eigen::AngleAxis<T> aa(eR);
            Eigen::Matrix<T, 3, 1> er = aa.angle() * aa.axis();

            // 设置残差的前3个元素
            residual.head(3) = er;

            // 计算位置误差
            residual.template block<3, 1>(3, 0) = p_current - ndt_p_.cast<T>();

            // 应用信息矩阵
            residual = ndt_info_.cast<T>() * residual;

            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Matrix4d &pc_pose, Eigen::Matrix<double, 6, 6> ndt_info)
        {
            return (new ceres::AutoDiffCostFunction<
                    PointCloudFactor, 6, 4, 3>(new PointCloudFactor(pc_pose, ndt_info)));
        }

        // Eigen::Matrix4f pc_pose_;
        Eigen::Matrix<double, 6, 6> ndt_info_;
        Eigen::Quaterniond ndt_q_;
        Eigen::Vector3d ndt_p_;
    };

};

#endif