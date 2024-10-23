
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include "process.h"


int main(int argc, char ** argv)
{
    ros::init(argc,argv,"slam");
    

    std::string imu_topic = "/imu/data";
    std::string point_cloud_topic = "/velodyne_points";

    slam_czc::Process process(imu_topic,point_cloud_topic);



    ROS_INFO("slam start");
    ros::spin();


}