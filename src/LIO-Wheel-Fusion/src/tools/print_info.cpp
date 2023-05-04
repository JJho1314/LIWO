#include "tools/print_info.hpp"
#include "glog/logging.h"

void PrintInfo::PrintPose(std::string head, Eigen::Matrix4f pose) {
    Eigen::Affine3f aff_pose;
    aff_pose.matrix() = pose;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(aff_pose, x, y, z, roll, pitch, yaw);
    LOG(INFO) << head << " x: " << x << "\ty: " << y << "\tz: " << z << "\troll: " << roll * 180 / M_PI << "\tpitch: "
              << pitch * 180 / M_PI << "\tyaw: " << yaw * 180 / M_PI;
//        std::cout << head
//                  << x << "," << y << "," << z << ","
//                  << roll * 180 / M_PI << "," << pitch * 180 / M_PI << "," << yaw * 180 / M_PI
//                  << std::endl;
}