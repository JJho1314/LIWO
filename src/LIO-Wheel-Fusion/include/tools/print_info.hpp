#ifndef SRC_TOOLS_PRINT_INFO_HPP_
#define SRC_TOOLS_PRINT_INFO_HPP_

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "pcl/common/eigen.h"

class PrintInfo {
public:
    static void PrintPose(std::string head, Eigen::Matrix4f pose);
};

#endif