#ifndef IMU_PROCESS_H
#define IMU_PROCESS_H

#include "utility.h"
#include <deque>
#include "imu_init.h"
#include "imu_preintegration.h"
#include "ceres/ceres.h"
#include "factor.h"
#include "parameters.h"


namespace slam_czc
{
    class Process
    {
    private:
        /* data */
    public:
        Process(std::string imu_topic, std::string pc_topic)
        {
            sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 2000, &Process::processIMU, this, ros::TransportHints().tcpNoDelay()); // 频率100

            sub_point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(pc_topic, 10, &Process::processPointCloud, this, ros::TransportHints().tcpNoDelay()); // 频率10
            
            pub_path_ = nh.advertise<nav_msgs::Path>("/path", 1);
            /*
nh.subscribe：这是 ros::NodeHandle 的一个方法，用于创建一个订阅者对象。
<sensor_msgs::Imu>：指定订阅的话题类型为 sensor_msgs::Imu。
imuTopic：这是话题的名称，通常是一个字符串变量。
2000：队列大小，即订阅者缓存的消息数量。如果消息发送速率高于处理速率，最多缓存 2000 条消息。
&ImageProjection::imuHandler：回调函数的指针，ImageProjection 是类名，imuHandler 是类中的成员函数。
this：指向当前类实例的指针。
ros::TransportHints().tcpNoDelay()：优化选项，用于设置 TCP 连接的 no_delay 选项，减少数据延迟。
ros::TransportHints 详解
ros::TransportHints 是一个类，提供了多种通信优化选项。在你的示例中，tcpNoDelay() 方法用于设置 TCP 连接的 no_delay 选项，这可以减少数据传输的延迟。

tcpNoDelay 选项
TCP 协议有一个 Nagle 算法，默认情况下，当发送小数据包时，TCP 会等待一段时间，收集更多的数据后再一起发送，以减少网络传输次数。这对于某些应用是有利的，但对于高频率的实时数据传输（如 IMU 数据），这种行为会导致不必要的延迟。

通过设置 tcpNoDelay 选项，可以禁用 Nagle 算法，使得每个数据包立即发送，从而减少延迟。
*/

            // 在ROS中，当你订阅一个话题时，通常会接收到一个智能指针（如 std::shared_ptr 或 boost::shared_ptr），这个智能指针指向一个消息对象。在这种情况下，sensor_msgs::PointCloud2::ConstPtr 是一个指向 sensor_msgs::PointCloud2 消息的常量智能指针。
            TIL_ = TIC;
            QIL_.fromRotationMatrix(RIC);
            ndt_.setResolution(1.0);             // NDT网格分辨率
            ndt_.setMaximumIterations(35);       // 最大迭代次数
            ndt_.setTransformationEpsilon(0.01); // 收敛阈值
            ndt_.setStepSize(0.1);               // 梯度下降步长
            ndt_.setNumThreads(4);               // 使用多线程加速
        }

        void addIMU(slam_czc::imu_ptr imu)
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



        ros::NodeHandle nh;
        ros::Subscriber sub_imu;
        ros::Subscriber sub_point_cloud;
        ros::Publisher pub_path_;
        
        std::string odom_frame_id = "odom";


        nav_msgs::Path global_path;
        
        std::shared_ptr<IMUPreintegration> imu_pre_ = nullptr;
        State current_state_;
        State last_state_;
        std::deque<slam_czc::imu_ptr> imu_deque_;
        std::deque<slam_czc::PointType::Ptr> pc_deque_;
        double last_timestamp_imu_ = -1;
        bool pc_pushed_ = false;

        SortedData sorted_data_;
        bool imu_init_flag = false;
        StaticIMUInit imu_init_;
        double last_timestamp_pc_ = -1;
        IMUPtr last_imu_ = nullptr;

        bool pc_first_flag_ = true;
        Eigen::Vector3d TIL_{-0.17, 0, 0.255};
        Eigen::Quaterniond QIL_ = Eigen::Quaterniond::Identity();
        pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
        Eigen::Matrix4f ndt_pose_;
        Eigen::Matrix<double,15,15> prior_info_ = Eigen::Matrix<double,15,15>::Identity();
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
    };

}

#endif