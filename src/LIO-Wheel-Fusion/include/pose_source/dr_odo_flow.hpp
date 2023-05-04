
#ifndef SRC_POSE_SOURCE_DR_ODO_FLOW_HPP_
#define SRC_POSE_SOURCE_DR_ODO_FLOW_HPP_

#include <ros/ros.h>
// self-defined rosmsg
#include "customized_msgs/pose_source.h"
// subscriber
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/velocity_subscriber.hpp"
#include "subscriber/odom_subscriber.hpp"
// publisher
#include "publisher/pose_source_publisher.hpp"
#include "publisher/tf_broadcaster.hpp"
#include "publisher/path_publisher.hpp"
// tools
#include <yaml-cpp/yaml.h>
#include "tools/tic_toc.hpp"
#include "ahrs/Estimator.h"

class DrOdoFlow {
public:
    DrOdoFlow(ros::NodeHandle &nh);

    void Run();

    void ReadData();

    void updateDrOdo();

    Eigen::Matrix4d Fusing_IMU_VelOdom(IMUData imu_front, IMUData imu_back);

    void WheelVel2IMUVel(VelocityData &vel_data, const Eigen::Vector3d &ang_vel);

    Eigen::Vector3d updateU(std::deque<VelocityData> &unsynced_velocity, double front_time, double back_time, const Eigen::Vector3d &ang_vel);
    Eigen::Vector3d updateU(std::deque<VelocityData> &unsynced_velocity, double front_time, double back_time);

    void PublishData();

private:
    // Subscriber
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<OdomSubscriber> vel_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> sjtu_vel_sub_ptr_;
    // Publisher
    // for use
    std::shared_ptr<PoseSourcePublisher> dr_odo_pub_ptr_;
    // for rviz
    std::shared_ptr<TFBroadCaster> tf_pub_ptr_;
    std::shared_ptr<PathPublisher> path_pub_ptr_;
    // Orientation Estimator
    std::shared_ptr<OriEst::Estimator> orientation_estimator;
    // Info Container
    std::deque<IMUData> raw_imu_;
    std::deque<IMUData> unsynced_imu_;
    std::deque<VelocityData> unsynced_velocity_;
    std::deque<customized_msgs::pose_source> dr_buffer;
    // Visualization
    Eigen::Matrix4d Twi;
};

#endif //SRC_POSE_SOURCE_DR_ODO_FLOW_HPP_


/*
#ifndef SRC_POSE_SOURCE_DR_ODO_FLOW_HPP_
#define SRC_POSE_SOURCE_DR_ODO_FLOW_HPP_

#include <ros/ros.h>
// self-defined rosmsg
#include "customized_msgs/pose_source.h"
// subscriber
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/velocity_subscriber.hpp"
// publisher
#include "publisher/pose_source_publisher.hpp"
#include "publisher/tf_broadcaster.hpp"
#include "publisher/path_publisher.hpp"
// tools
#include <yaml-cpp/yaml.h>
#include "tools/tic_toc.hpp"
#include "ahrs/Estimator.h"

class DrOdoFlow {
public:
    DrOdoFlow(ros::NodeHandle &nh);

    void Run();

    void ReadData();

    void updateDrOdo();

    Eigen::Matrix4d Fusing_IMU_VelOdom(IMUData imu_front, IMUData imu_back);

    Eigen::Vector3d updateU(std::deque<VelocityData> &unsynced_velocity, double front_time, double back_time);

    void PublishData();

private:
    // Subscriber
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> vel_sub_ptr_;
    // Publisher
    // for use
    std::shared_ptr<PoseSourcePublisher> dr_odo_pub_ptr_;
    // for rviz
    std::shared_ptr<TFBroadCaster> tf_pub_ptr_;
    std::shared_ptr<PathPublisher> path_pub_ptr_;
    // Orientation Estimator
    std::shared_ptr<OriEst::Estimator> orientation_estimator;
    // Info Container
    std::deque<IMUData> raw_imu_;
    std::deque<IMUData> unsynced_imu_;
    std::deque<VelocityData> unsynced_velocity_;
    std::deque<customized_msgs::pose_source> dr_buffer;
    // Visualization
    Eigen::Matrix4d Twi;
};

#endif //SRC_POSE_SOURCE_DR_ODO_FLOW_HPP_
*/