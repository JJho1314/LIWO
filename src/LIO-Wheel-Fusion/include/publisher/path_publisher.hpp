#ifndef SRC_PUBLISHER_PATH_PUBLISHER_HPP_
#define SRC_PUBLISHER_PATH_PUBLISHER_HPP_

#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor_data/pose_data.hpp"

class PathPublisher {
public:
    PathPublisher(ros::NodeHandle &nh,
                  std::string topic_name,
                  std::string frame_id,
                  int buff_size);

    PathPublisher() = default;

    void Publish(PoseData &pose);

    void Publish(const Eigen::Matrix4f &pose, double time);

    void Publish(const Eigen::Matrix4d &pose, double time);

    bool HasSubscribers();

    void Reset();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";

    uint32_t index_;
    nav_msgs::Path mPath;
};

#endif