#ifndef FACTOR_H
#define FACTOR_H
#include "utility.h"
#include "imu_preintegration.h"
#include "ceres/ceres.h"

namespace slam_czc
{

    struct IMUFactor
    {
        IMUFactor(std::shared_ptr<IMUPreintegration> imu_pre,
                  const Eigen::Vector3d g)
            : imu_pre_(imu_pre), g_(g), dt_(imu_pre.dt_) { sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(imu_pre_->cov_.inverse()).matrixL().transpose(); }

        template <typename T>
        // last_q_, last_p_, last_v_,  last_bg_, last_ba_,current_q_, current_p_, current_v_,current_bg_, current_ba_
        bool operator()(const T *last_q_, const T *last_p_, const T *last_v_, const T *last_bg_, const T *last_ba_, const T *current_q_, const T *current_p_, const T *current_v_, const T *current_bg_, const T *current_ba_, T *residuals) const
        {
            Eigen::Map<const Eigen::Quaterniond> q_last(last_q_);
            Eigen::Map<const Eigen::Vector3d> p_last(last_p_);
            Eigen::Map<const Eigen::Vector3d> v_last(last_v_);
            Eigen::Map<const Eigen::Vector3d> bg_last(last_bg_);
            Eigen::Map<const Eigen::Vector3d> ba_last(last_ba_);
            Eigen::Map<const Eigen::Quaterniond> q_current(current_q_);
            Eigen::Map<const Eigen::Vector3d> p_current(current_p_);
            Eigen::Map<const Eigen::Vector3d> v_current(current_v_);
            Eigen::Map<const Eigen::Vector3d> bg_current(current_bg_);
            Eigen::Map<const Eigen::Vector3d> ba_current(current_ba_);
            Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);

            const Eigen::Vector3d delta_p = imu_pre_->getDeltaPosition(bg, ba);
            const Eigen::Vector3d delta_v = imu_pre_->getDeltaVelocity(bg, ba);
            const Eigen::Quaterniond delta_q = imu_pre_->getDeltaRotation(bg);
            const Eigen::Vector3d delta_R = delta_q.ToRotationMatrix();

            Eigen::Matrix3d R_last = q_last.ToRotationMatrix().transpose();
            Eigen::Matrix3d eR = delta_R.transpose() * R_last * q_current.ToRotationMatrix().transpose(); // 两时刻的姿态与预积分姿态应该一致
            Eigen::AngleAxisd aa(eR);
            Eigen::Vector3d er = aa.angle() * aa.axis();
            residual.head(3) = er;

            const Eigen::Vector3d ev = R_last * (v_current - v_last - g_ * dt_) - delta_v; // 速度差转到预积分坐标下与预积分比较

            residual.block<3, 1>(3, 0) = ev;

            const Eigen::Vector3d ep = R_last * (p_current - p_last - v_last * dt_ - g_ * dt_ * dt_ / 2) - delta_p;
            residual.block<6, 1>(3, 0) = ep;

            residual.block<9, 1>(3, 0) = bg_current - bg_last;
            residual.block<12, 1>(3, 0) = ba_current - ba_last;

            //  这就是带有信息矩阵的残差
            residual = sqrt_info * residual;

            return true;
        }

        static ceres::CostFunction *Create(const std::shared_ptr<IMUPreintegration> imu_pre_,
                                           const Eigen::Vector3d g)
        {
            return (new ceres::AutoDiffCostFunction<
                    IMUFactor, 15, 4, 3, 3, 3, 3, 4, 3, 3, 3, 3>( // 第一个参数是CostFunction，第二个参数是残差的维数，第三个参数是参数块的维数，上一次旋转矩阵4维，平移3维，速度3维，零偏两个3维，这次的旋转4维、平移、速度3维、零偏两个3维
                new IMUFactor(pre, g)));
        }

        std::shared_ptr<IMUPreintegration> imu_pre_ = nullptr;
        Eigen::Vector3d g_;
        double dt_ = 0;
        Eigen::Matrix<double, 15, 15> sqrt_info;
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
            Eigen::Map<const Eigen::Quaterniond> q_last(q);
            Eigen::Map<const Eigen::Vector3d> p_last(p);
            Eigen::Map<const Eigen::Vector3d> v_last(v);
            Eigen::Map<const Eigen::Vector3d> bg_last(bg);
            Eigen::Map<const Eigen::Vector3d> ba_last(ba);
            Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);

            Eigen::Matrix3d eR = state_.q_.inverse().toRotationMatrix() * q_last.toRotationMatrix(); //
            Eigen::AngleAxisd aa(eR);
            Eigen::Vector3d er = aa.angle() * aa.axis();
            residual.head(3) = er;

            residual.block<3, 1>(3, 0) = p_last - state_.p_;

            residual.block<3, 1>(6, 0) = v_last - state_.v_;

            residual.block<3, 1>(9, 0) = bg_last - state_.bg_;
            residual.block<3, 1>(12, 0) = ba_last - state_.ba_;
            residual = prior_info_ * residual;
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
    }

    struct PointCloudFactor
    {
        PointCloudFactor(const Eigen::Matrix4f &pc_pose, const Eigen::Matrix<double, 6, 6> &ndt_info)
        {
            ndt_info_ = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(ndt_info.inverse()).matrixL().transpose();
            // 提取平移部分
            ndt_p_ = pc_pose.block<3, 1>(0, 3);

            // 提取旋转部分为旋转矩阵
            Eigen::Matrix3f rotation_matrix = transform.block<3, 3>(0, 0);

            // 将旋转矩阵转换为四元数
            ndt_q_.fromRotationMatrix(rotation_matrix);
        }

        template <typename T>
        bool operator()(const T *q, const T *p, T *residuals) const
        {
            Eigen::Map<const Eigen::Quaterniond> q_current(q);
            Eigen::Map<const Eigen::Vector3d> p_current(p);
            Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);

            Eigen::Matrix3d eR = ndt_q_.inverse().toRotationMatrix() * q_current.toRotationMatrix();
            Eigen::AngleAxisd aa(eR);
            Eigen::Vector3d er = aa.angle() * aa.axis();
            residual.head(3) = er;
            residual.block<3, 1>(3, 0) = p_current - ndt_p_;
            residual = ndt_info_ * residual;
            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Matrix4f &pc_pose, Eigen::Matrix<double, 6, 6> ndt_info)
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