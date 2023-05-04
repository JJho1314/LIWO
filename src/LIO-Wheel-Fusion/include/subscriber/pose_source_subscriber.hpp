//
// Created by warren on 2021/12/28.
//
#ifndef SRC_SUBSCRIBER_POSE_SOURCE_SUBSCRIBER_HPP_
#define SRC_SUBSCRIBER_POSE_SOURCE_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "customized_msgs/pose_source.h"

class PoseSourceSubscriber {
public:
    PoseSourceSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);

    PoseSourceSubscriber() = default;

    std::deque<customized_msgs::pose_source> ParseData(std::deque<customized_msgs::pose_source> &target_deque_edge);

private:
    void msg_callback(const customized_msgs::pose_source &edge);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<customized_msgs::pose_source> new_edge_data_;

    std::mutex buff_mutex_;
};


#endif //SRC_SUBSCRIBER_POSE_SOURCE_SUBSCRIBER_HPP_
