#ifndef INTEGRATION_BASE_H
#define INTEGRATION_BASE_H

#include "utility.h"

class IntegrationBase
{
public:
    IntegrationBase() = delete;
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg, const Eigen::Vector3d &_cov_acce_n,
                    const Eigen::Vector3d &_cov_gyro_n, const Eigen::Matrix3d &_cov_acce_w, const Eigen::Matrix3d &_cov_gyro_w)
        : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
          linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
          jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
          sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}

    {
        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0) = _cov_acce_n[0] * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) = _cov_gyro_n[0] * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) = _cov_acce_n[0] * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) = _cov_gyro_n[0] * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) = _cov_acce_w;
        noise.block<3, 3>(15, 15) = _cov_gyro_w;
    }

    /**
     * @brief 根据新设置的imu零偏重新对该帧进行预积分
     *
     * @param[in] _linearized_ba
     * @param[in] _linearized_bg
     */

    void midPointIntegration(double _dt,
                             const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                             const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                             const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                             Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                             Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
        // ROS_INFO("midpoint integration");
        //  首先中值积分更新状态量
        Eigen::Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
        Eigen::Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
        Eigen::Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        result_delta_v = delta_v + un_acc * _dt;
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;
        // 随后更新方差矩阵及雅克比
        if (update_jacobian)
        {
            Eigen::Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Eigen::Vector3d a_0_x = _acc_0 - linearized_ba;
            Eigen::Vector3d a_1_x = _acc_1 - linearized_ba;
            Eigen::Matrix3d R_w_x, R_a_0_x, R_a_1_x;

            R_w_x << 0, -w_x(2), w_x(1),
                w_x(2), 0, -w_x(0),
                -w_x(1), w_x(0), 0;
            R_a_0_x << 0, -a_0_x(2), a_0_x(1),
                a_0_x(2), 0, -a_0_x(0),
                -a_0_x(1), a_0_x(0), 0;
            R_a_1_x << 0, -a_1_x(2), a_1_x(1),
                a_1_x(2), 0, -a_1_x(0),
                -a_1_x(1), a_1_x(0), 0;

            Eigen::MatrixXd F = Eigen::MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Eigen::Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3, 3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * Eigen::MatrixXd::Identity(3, 3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                                  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Eigen::Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();
            // cout<<"A"<<endl<<A<<endl;

            Eigen::MatrixXd V = Eigen::MatrixXd::Zero(15, 18);
            V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) = 0.5 * Eigen::MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(3, 9) = 0.5 * Eigen::MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * _dt;
            V.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * _dt;

            // step_jacobian = F;
            // step_V = V;
            jacobian = F * jacobian;
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
        }
    }

    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;
        Eigen::Vector3d result_delta_p;
        Eigen::Quaterniond result_delta_q;
        Eigen::Vector3d result_delta_v;
        Eigen::Vector3d result_linearized_ba;
        Eigen::Vector3d result_linearized_bg;

        midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 1);

        // checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
        //                     linearized_ba, linearized_bg);
        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();
        sum_dt += dt;
        acc_0 = acc_1;
        gyr_0 = gyr_1;
    }

    slam_czc::State predict(slam_czc::State start, Eigen::Vector3d g)
    {
        Eigen::Quaterniond qj = Eigen::Quaterniond(start.q_.toRotationMatrix() * delta_q.toRotationMatrix());
        Eigen::Vector3d vj = start.q_.toRotationMatrix() * delta_v + start.v_ + g * sum_dt;
        Eigen::Vector3d pj = start.q_.toRotationMatrix() * delta_p + start.p_ + start.v_ * sum_dt + 0.5d * g * sum_dt * sum_dt;

        auto state = slam_czc::State(start.timestamp_ + sum_dt, qj, pj, vj, linearized_bg, linearized_ba);

        return state;
    }


    // 修改 getDeltaRotation 方法签名以支持 Eigen::MatrixBase
    template <typename Derived>
    Eigen::Quaternion<typename Derived::Scalar> getDeltaRotation(const Eigen::MatrixBase<Derived> &bg) const
    {
        using ScalarType = typename Derived::Scalar;
        Eigen::Matrix<ScalarType, 3, 3> dq_dbg = jacobian.block<3, 3>(3, 12).cast<ScalarType>();
        Eigen::Matrix<ScalarType, 3, 1> tem = dq_dbg * (bg - linearized_bg.cast<ScalarType>());
        return delta_q.cast<ScalarType>() * slam_czc::theta2Q(tem);
    }

    // 修改 getDeltaVelocity 方法签名以支持 Eigen::MatrixBase
    template <typename Derived1, typename Derived2>
    Eigen::Matrix<typename Derived1::Scalar, 3, 1> getDeltaVelocity(const Eigen::MatrixBase<Derived1> &bg, const Eigen::MatrixBase<Derived2> &ba) const
    {
        using ScalarType = typename Derived1::Scalar;
        Eigen::Matrix<ScalarType, 3, 3> dv_dba = jacobian.block<3, 3>(6, 9).cast<ScalarType>();
        Eigen::Matrix<ScalarType, 3, 3> dv_dbg = jacobian.block<3, 3>(6, 12).cast<ScalarType>();
        return delta_v.cast<ScalarType>() + dv_dbg * (bg - linearized_bg.cast<ScalarType>()) + dv_dba * (ba - linearized_ba.cast<ScalarType>());
    }

    // 修改 getDeltaPosition 方法签名以支持 Eigen::MatrixBase
    template <typename Derived1, typename Derived2>
    Eigen::Matrix<typename Derived1::Scalar, 3, 1> getDeltaPosition(const Eigen::MatrixBase<Derived1> &bg, const Eigen::MatrixBase<Derived2> &ba) const
    {
        using ScalarType = typename Derived1::Scalar;
        Eigen::Matrix<ScalarType, 3, 3> dp_dba = jacobian.block<3, 3>(0, 9).cast<ScalarType>();
        Eigen::Matrix<ScalarType, 3, 3> dp_dbg = jacobian.block<3, 3>(0, 12).cast<ScalarType>();
        return delta_p.cast<ScalarType>() + dp_dbg * (bg - linearized_bg.cast<ScalarType>()) + dp_dba * (ba - linearized_ba.cast<ScalarType>());
    }

    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;

    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 15, 15> step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Eigen::Matrix<double, 18, 18> noise;

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;
};

#endif
