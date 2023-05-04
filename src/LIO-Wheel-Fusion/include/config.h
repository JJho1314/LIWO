//
// Created by warren on 2022/1/5
//

#ifndef SRC_CONFIG_H
#define SRC_CONFIG_H

#include <yaml-cpp/yaml.h>
#include "global_definition/global_definition.h"
#include <Eigen/Core>
#include <vector>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>      // ifstream
#include <stdio.h>
#include <unistd.h>

using namespace std;

class Config {

public:

    Config() {}

    static void readConfig();

public:
    // common
    static string lid_topic;
    static string imu_topic;
    static string vel_topic;
    static string sjtu_vel_topic;
    static string cam_topic;
    static float time_per_lidar;
    static bool use_imu_time;
    static bool use_sjtu_config;

    // pose source
    // topics
    // for use
    static string dr_odo_topic;
    // for rviz
    static string dr_odo_tf_topic;
    static string dr_odo_path_topic;

    // distortion removal
    // topics
    static string distort_cloud_topic;
    static string undistort_cloud_topic;
    // params for checking
    static bool check_undistort;
    static bool save_cloud;
    static size_t startSaving_frameCnt;


    // mapping
    static Eigen::Matrix4f lidar_to_imu_; // to指坐标变换
    static Eigen::Matrix4f imu_to_lidar_;
    static Eigen::Matrix4d wheel_to_imu_;
    static Eigen::Matrix4d imu_to_wheel_;


    static YAML::Node config_node_;

    static float FREQ;
    static bool USE_EVO_APRILTAG;
    static float td;
    static Eigen::Matrix3d ric;
    static Eigen::Vector3d tic;
    static std::string cam_param;

};


#endif //SRC_CONFIG_H
