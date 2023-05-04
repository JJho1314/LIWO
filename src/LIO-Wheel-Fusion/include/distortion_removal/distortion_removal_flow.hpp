//
// Created by warren on 2022/1/4.
//

#ifndef SRC_DISTORTION_REMOVAL_DISTORTION_REMOVAL_FLOW_HPP_
#define SRC_DISTORTION_REMOVAL_DISTORTION_REMOVAL_FLOW_HPP_

#include <ros/ros.h>
// self-defined rosmsg
#include "customized_msgs/pose_source.h"
// subscriber
#include "subscriber/pose_source_subscriber.hpp"
#include "subscriber/cloud_subscriber.hpp"
// publisher
#include "publisher/pose_source_publisher.hpp"
#include "publisher/tf_broadcaster.hpp"
#include "publisher/path_publisher.hpp"
#include "publisher/cloud_publisher.hpp"
// tools
#include <yaml-cpp/yaml.h>
#include "tools/tic_toc.hpp"
#include "ahrs/Estimator.h"

class DistortRemovalFlow {
public:
    DistortRemovalFlow(ros::NodeHandle &nh);

    void Run();

    void ReadData();

    void getCurCloud();

    void updatePred();

    void removeDistortion();

    void PublishData();

    void saveCloud(); // for checking

private:
    // Subscriber
    std::shared_ptr<PoseSourceSubscriber> dr_sub_ptr_;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    // Publisher
    std::shared_ptr<CloudPublisher> distort_cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> undistort_cloud_pub_ptr_;
    // Flags and cnts
    bool canRemoveDistort;
    size_t count = 0;
    // Info Container
    std::deque<customized_msgs::pose_source> raw_dr_buffer_;
    std::deque<customized_msgs::pose_source> dr_pred_buffer_;
    std::deque<CloudData> cloud_data_buffer_;
    CloudData current_cloud_data_;
    CloudData undistort_cloud_data_;
    Eigen::Matrix4f T_pred;
};

#endif //SRC_DISTORTION_REMOVAL_DISTORTION_REMOVAL_FLOW_HPP_
