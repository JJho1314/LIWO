//
// Created by warren on 2021/12/28.
//
#ifndef SRC_PUBLISHER_POSE_SOURCE_PUBLISHER_HPP_
#define SRC_PUBLISHER_POSE_SOURCE_PUBLISHER_HPP_

#include <string>
#include <deque>
#include <ros/ros.h>
#include "customized_msgs/pose_source.h"

class PoseSourcePublisher {
public:
    PoseSourcePublisher(ros::NodeHandle &nh,
                        std::string topic_name,
                        size_t buff_size);

    PoseSourcePublisher() = default;

    bool HasSubscribers();

    void PublishData(const customized_msgs::pose_source edge);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
};

#endif //SRC_PUBLISHER_POSE_SOURCE_PUBLISHER_HPP_
