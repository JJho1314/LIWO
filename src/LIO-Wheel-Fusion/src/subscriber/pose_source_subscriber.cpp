//
// Created by warren on 2021/12/28.
//
#include "subscriber/pose_source_subscriber.hpp"
#include "glog/logging.h"
#include "tools/print_info.hpp"

PoseSourceSubscriber::PoseSourceSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &PoseSourceSubscriber::msg_callback, this);
}

void PoseSourceSubscriber::msg_callback(const customized_msgs::pose_source &edge) {
    buff_mutex_.lock();
    new_edge_data_.push_back(edge);
    buff_mutex_.unlock();
}

std::deque<customized_msgs::pose_source>
PoseSourceSubscriber::ParseData(std::deque<customized_msgs::pose_source> &target_deque_edge) {
    buff_mutex_.lock();
    std::deque<customized_msgs::pose_source> copy_data;
    if (new_edge_data_.size() > 0) {
        target_deque_edge.insert(target_deque_edge.end(), new_edge_data_.begin(), new_edge_data_.end());
        copy_data = new_edge_data_;
        new_edge_data_.clear();
    }
    buff_mutex_.unlock();
    return copy_data;
}