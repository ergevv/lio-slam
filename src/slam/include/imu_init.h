#ifndef IMU_INIT_H
#define IMU_INIT_H

#include "utility.h"

namespace slam_czc
{
    class StaticIMUInit
    {
    public:
        struct Options
        {
            Options() {}
            double init_need_time_ = 10;
            int init_imu_max_size_ = 2000;
            int init_imu_min_size_ = 2000;
            double max_gyro_var_ = 0.5;
            double max_acce_var_ = 0.05;
            double gravity_norm_ = 0.981;
        };

        StaticIMUInit(Options options = Options()) : options_(options)
        {
        }

        bool addIMU(const IMU &imu);
        bool tryInit();

    public:
        Options options_;
        bool init_success_ = false;
        std::deque<slam_czc::IMU> init_imu_deque_;
        double init_begin_time = 0;
        Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d cov_gyro_ = Eigen::Vector3d::Zero(); // 陀螺测量噪声协方差（初始化时评估）
        Eigen::Vector3d cov_acce_ = Eigen::Vector3d::Zero(); // 加计测量噪声协方差（初始化时评估）
        Eigen::Vector3d init_ba_ = Eigen::Vector3d::Zero(); 
        Eigen::Vector3d init_bg_ = Eigen::Vector3d::Zero(); 

    };
}

#endif