#ifndef IMU_PROCESS_H
#define IMU_PROCESS_H

#include "utility.h"
#include <deque>
#include "imu_init.h"
#include "integration_base.h"
#include "ceres/ceres.h"
#include "factor.h"
#include "parameters.h"
#include <nav_msgs/Path.h>

namespace slam_czc
{
    class Process
    {
    private:
        /* data */
    public:
        Process(std::string imu_topic, std::string pc_topic);
        void addIMU(slam_czc::ImuPtr imu)
        {
            last_timestamp_imu_ = imu->timestamp_;
            imu_deque_.push_back(imu);
        }

        void processIMU(const sensor_msgs::Imu::ConstPtr &imu_msg)
        {
            addIMU(convertToIMU(imu_msg));
        }

        bool dataAlign();
        bool dataProcess();

        void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
        {
            if (pc_msg->header.stamp.toSec() < last_timestamp_pc_)
            {
                ROS_WARN("data exception, clear pc_deque_");
                pc_deque_.clear();
            }
            last_timestamp_pc_ = pc_msg->header.stamp.toSec();
            PointType::Ptr cloud = convertToPCL(pc_msg);
            pc_deque_.push_back(cloud);
            // string
            // ROS_INFO("%d , %d",cloud->points);
            dataAlign();
        }

        bool initIMU();
        bool predictByIMU();
        bool alignByPC();
        bool optimize();
        bool vecter2double();
        void updatePath(const State &state);

        
        ros::NodeHandle nh;
        ros::Subscriber sub_imu;
        ros::Subscriber sub_point_cloud;
        ros::Publisher pub_path_;

        std::string odom_frame_id_ = "odom";

        nav_msgs::Path global_path_;

        std::shared_ptr<IntegrationBase> imu_pre_ = nullptr;
        State current_state_;
        State last_state_;
        std::deque<slam_czc::ImuPtr> imu_deque_;
        std::deque<slam_czc::PointType::Ptr> pc_deque_;
        double last_timestamp_imu_ = -1;
        bool pc_pushed_ = false;

        SortedData sorted_data_;
        bool imu_init_flag = false;
        StaticIMUInit imu_init_;
        double last_timestamp_pc_ = -1;
        ImuPtr last_imu_ = nullptr;

        bool pc_first_flag_ = true;
        Eigen::Vector3d TIL_{-0.17, 0, 0.255};
        Eigen::Quaterniond QIL_ = Eigen::Quaterniond::Identity();
        pcl::NormalDistributionsTransform<Point, Point> ndt_;  //模板得是点的类型`
        Eigen::Matrix4f ndt_pose_;
        Eigen::Matrix<double, 15, 15> prior_info_ = Eigen::Matrix<double, 15, 15>::Identity();
        double current_p_[3];
        double current_q_[4];
        double current_v_[3];
        double current_bg_[3];
        double current_ba_[3];

        double last_p_[3];
        double last_q_[4];
        double last_v_[3];
        double last_bg_[3];
        double last_ba_[3];

        Eigen::Vector3d g_{0.0, 0.0, 9.8};
        double acc_n_, acc_w_;
        double gyr_n_, gyr_w_;
    };

}

#endif