#include "parameters.h"

Eigen::Matrix3d RIC;
Eigen::Vector3d TIC;
Eigen::Vector3d G{0.0, 0.0, 9.8};
double ACC_N, ACC_W;
double GYR_N, GYR_W;


template <typename T>
T readRosParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}
bool readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readRosParam<std::string>(n, "config_file");
    auto yaml = YAML::LoadFile(config_file);
    std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
    std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
    ACC_N = yaml["acc_n"].as<double>();
    ACC_W = yaml["acc_w"].as<double>();
    GYR_N = yaml["gyr_n"].as<double>();
    GYR_W = yaml["gyr_w"].as<double>();
    G.z() = yaml["g_norm"].as<double>();
    TIC = slam_czc::vecFromArray(ext_t);
    RIC = slam_czc::matFromArray(ext_r);
    return true;
}