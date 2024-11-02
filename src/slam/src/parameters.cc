#include "parameters.h"

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

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

    TIC = vecFromArray(ext_t);
    RIC = matFromArray(ext_r);
    return true;
}