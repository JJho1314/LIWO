//
// Created by warren on 2021/12/28.
//
#include "publisher/pose_source_publisher.hpp"
#include "glog/logging.h"

PoseSourcePublisher::PoseSourcePublisher(ros::NodeHandle &nh,
                                         std::string topic_name,
                                         size_t buff_size)
        : nh_(nh) {
    publisher_ = nh_.advertise<customized_msgs::pose_source>(topic_name, buff_size);
}

void PoseSourcePublisher::PublishData(const customized_msgs::pose_source edge) {
    publisher_.publish(edge);
}

bool PoseSourcePublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}