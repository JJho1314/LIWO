#include "subscriber/velocity_subscriber.hpp"

#include "glog/logging.h"

VelocitySubscriber::VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size, bool use_vel_time)
        : nh_(nh), use_vel_time(use_vel_time) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::msg_callback, this);
    LOG(INFO) << "VelocitySubscriber topic_name: " << topic_name << std::endl;
}

void VelocitySubscriber::msg_callback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr) {
    buff_mutex_.lock();
    VelocityData velocity_data;
    if (use_vel_time)
        velocity_data.time = twist_msg_ptr->header.stamp.toSec();
    else
        velocity_data.time = ros::Time::now().toSec();
//        LOG(INFO) << std::fixed << "velocity_data.time: " << velocity_data.time;

    velocity_data.linear_velocity.x = twist_msg_ptr->twist.linear.x;
    velocity_data.linear_velocity.y = twist_msg_ptr->twist.linear.y;
    velocity_data.linear_velocity.z = twist_msg_ptr->twist.linear.z;

    velocity_data.angular_velocity.x = twist_msg_ptr->twist.angular.x;
    velocity_data.angular_velocity.y = twist_msg_ptr->twist.angular.y;
    velocity_data.angular_velocity.z = twist_msg_ptr->twist.angular.z;

    if (abs(velocity_data.linear_velocity.x) > 0.0 || abs(velocity_data.linear_velocity.y) > 0.0 ||
        abs(velocity_data.linear_velocity.z) > 0.0)
        nonZeroLast = velocity_data;

    new_velocity_data_.push_back(velocity_data);

    buff_mutex_.unlock();
}

void VelocitySubscriber::ParseData(std::deque<VelocityData> &velocity_data_buff) {
    buff_mutex_.lock();
    if (!new_velocity_data_.empty()) {
        velocity_data_buff.insert(velocity_data_buff.end(), new_velocity_data_.begin(), new_velocity_data_.end());
        new_velocity_data_.clear();
    }
    buff_mutex_.unlock();
}