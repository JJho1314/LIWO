#ifndef SRC_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define SRC_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

#include "sensor_data/velocity_data.hpp"
#include <geometry_msgs/TwistStamped.h>

class VelocitySubscriber {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size, bool use_vel_time);

    VelocitySubscriber() = default;

    void ParseData(std::deque<VelocityData> &deque_velocity_data);

private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr);

public:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    bool use_vel_time;
    std::deque<VelocityData> new_velocity_data_;
    std::mutex buff_mutex_;
    VelocityData nonZeroLast;
};

#endif