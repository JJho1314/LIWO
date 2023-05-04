
#ifndef SRC_EVO_APRILTAG_H
#define SRC_EVO_APRILTAG_H

#include <stdio.h>
#include <deque>
#include <queue>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "calc_cam_pose/calcCamPose.h"

using namespace std;

const int WINDOW_SIZE = 10;
//std::vector<std::pair<double, Eigen::Matrix4d>> FusedPath;///for apriltag

void AprilTag_buf_push(const double &time, const cv::Mat &image, bool init);
void AprilTag_buf_filter(const double &first_time);
bool EVO(const double &total_length, deque<std::pair<double, Eigen::Matrix4d>> &FusedPath_start, deque<std::pair<double, Eigen::Matrix4d>> &FusedPath_latest);///for apriltag);
static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R);
#endif //SRC_EVO_APRILTAG_H
