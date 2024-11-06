#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <yaml-cpp/yaml.h>
#include "utility.h"

extern Eigen::Matrix3d RIC;
extern Eigen::Vector3d TIC;
extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;
extern Eigen::Vector3d G;

bool readParameters(ros::NodeHandle &n);
#endif

