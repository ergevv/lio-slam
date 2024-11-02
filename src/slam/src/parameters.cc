#include "parameters.h"

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
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
    auto yaml = YAML::LoadFile(yaml_file);
    std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
    std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    TIC = vecFromArray(ext_t);
    RIC = matFromArray(ext_r);
    return true;
}