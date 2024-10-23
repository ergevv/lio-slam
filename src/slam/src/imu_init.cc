#include "imu_init.h"

namespace slam_czc
{
    bool StaticIMUInit::addIMU(const IMU& imu)
    {
        if(init_success_)
        {
            return true;
        }
        if(init_imu_deque_.empty())
        {
            init_begin_time = imu.timestamp_;
        }

        init_imu_deque_.push_back(imu);

        //imu需要足够长的静止时间
        double static_duration = imu.timestamp_ - init_begin_time;
        if (static_duration > options_.init_need_time_)
        {
            tryInit();
        }

        while (init_imu_deque_.size() > options_.init_imu_max_size_)
        {
            init_imu_deque_.pop_front();
        }

        return true;
    }


    bool StaticIMUInit::tryInit()
    {
        if(init_imu_deque_.size() < options_.init_imu_min_size_)
        {
            return false;
        }

        Eigen::Vector3d mean_gyro, mean_acce;
        computeMeanAndCov(init_imu_deque_, mean_gyro, cov_gyro_, [](const IMU& imu){return imu.gyro_;});  //无->,代表没有显式指定返回类型，那么编译器会根据return语句的结果自动推导出返回类型
        computeMeanAndCov(init_imu_deque_, mean_acce, cov_acce_, [](const IMU& imu){return imu.acce_;});

        //重力与加速度方向相反
        gravity_ = -mean_acce/mean_acce.norm() * options_.gravity_norm_;
        //计算去除重力后的协方差
        computeMeanAndCov(init_imu_deque_, mean_acce, cov_acce_,
                                [this](const IMU& imu) { return imu.acce_ + gravity_; });
        if(cov_gyro_.norm()>options_.max_gyro_var_ || cov_acce_.norm()>options_.max_acce_var_)
        {
            ROS_INFO("测量噪声太大");
            return false;
        }
        
        //零偏
        init_bg_ = mean_gyro;
        init_ba_ = mean_acce;
        init_success_ = true;
        return true;
    }


}
