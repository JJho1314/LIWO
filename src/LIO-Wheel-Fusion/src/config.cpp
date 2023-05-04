#include "config.h"
#include "CommonFunc.h"

using namespace std;

YAML::Node Config::config_node_ = YAML::Node();
// common
string Config::lid_topic;
string Config::imu_topic;
string Config::vel_topic;
string Config::sjtu_vel_topic;
string Config::cam_topic;
float Config::time_per_lidar;

// pose source
// topics
// for use
string Config::dr_odo_topic;
// for rviz
string Config::dr_odo_tf_topic;
string Config::dr_odo_path_topic;

// distortion removal
// topics
string Config::distort_cloud_topic;
string Config::undistort_cloud_topic;
// params for checking
bool Config::check_undistort;
bool Config::save_cloud;
bool Config::use_imu_time;
bool Config::use_sjtu_config;
size_t Config::startSaving_frameCnt;

// mapping
Eigen::Matrix4f Config::lidar_to_imu_;
Eigen::Matrix4f Config::imu_to_lidar_;
Eigen::Matrix4d Config::wheel_to_imu_;
Eigen::Matrix4d Config::imu_to_wheel_;

float Config::FREQ;
bool Config::USE_EVO_APRILTAG;

Eigen::Matrix3d Config::ric;
Eigen::Vector3d Config::tic;
float Config::td;
std::string Config::cam_param;

void Config::readConfig() {

    config_node_ = YAML::LoadFile(VELODYNE_YAML_PATH);

    // common
    YAML::Node common_config_node_ = config_node_["common"];
    lid_topic = common_config_node_["lid_topic"].as<string>();
    imu_topic = common_config_node_["imu_topic"].as<string>();
    vel_topic = common_config_node_["vel_topic"].as<string>();
    sjtu_vel_topic = common_config_node_["sjtu_vel_topic"].as<string>();
    cam_topic = common_config_node_["cam_topic"].as<string>();
    time_per_lidar = common_config_node_["time_per_lidar"].as<float>();
    use_imu_time = common_config_node_["use_imu_time"].as<bool>();
    use_sjtu_config = common_config_node_["use_sjtu_config"].as<bool>();

    // pose source
    YAML::Node pose_source_config_node_ = config_node_["pose_source"];
    // topics
    // for use
    dr_odo_topic = pose_source_config_node_["dr_odo_topic"].as<string>();
    // for rviz
    dr_odo_tf_topic = pose_source_config_node_["dr_odo_tf_topic"].as<string>();
    dr_odo_path_topic = pose_source_config_node_["dr_odo_path_topic"].as<string>();

    // distortion removal
    YAML::Node distortion_removal_config_node_ = config_node_["distortion_removal"];
    // topics
    distort_cloud_topic = distortion_removal_config_node_["distort_cloud_topic"].as<string>();
    undistort_cloud_topic = distortion_removal_config_node_["undistort_cloud_topic"].as<string>();
    // params for checking
    check_undistort = distortion_removal_config_node_["check_undistort"].as<bool>();
    save_cloud = distortion_removal_config_node_["save_cloud"].as<bool>();
    startSaving_frameCnt = distortion_removal_config_node_["startSaving_frameCnt"].as<size_t>();




    // mapping
    YAML::Node mapping_config_node_ = config_node_["mapping"];
    lidar_to_imu_.setIdentity();
    wheel_to_imu_.setIdentity();
    for (int i = 0; i < 3; i++)
        lidar_to_imu_(i, 3) = mapping_config_node_["imu_t_lidar"][i].as<float>();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            lidar_to_imu_(i, j) = mapping_config_node_["imu_R_lidar"][3 * i + j].as<float>();
        }
    }

    for (int i = 0; i < 3; i++)
        wheel_to_imu_(i, 3) = mapping_config_node_["imu_t_wheel"][i].as<double>();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            wheel_to_imu_(i, j) = mapping_config_node_["imu_R_wheel"][3 * i + j].as<double>();
        }
    }

    imu_to_wheel_ = wheel_to_imu_.inverse();

    imu_to_lidar_ = EigenIsoInv(lidar_to_imu_);

    /// for apriltag
    YAML::Node eva_node_ = config_node_["evaluation"];
    FREQ = eva_node_["freq"].as<float>();
    USE_EVO_APRILTAG = eva_node_["use_evo_AprilTag"].as<bool>();
    for (size_t i = 0; i < 3; i++){
        for (size_t j = 0; j < 3; j++)
            ric(i, j) = eva_node_["Cam2Imu_R"][3 * i + j].as<double>();
    }
    for (size_t j = 0; j < 3; j++)
        tic(j) = eva_node_["Cam2Imu_T"][j].as<double>();
    td = eva_node_["td"].as<float>();
    cam_param = eva_node_["cam_param"].as<std::string>();

}