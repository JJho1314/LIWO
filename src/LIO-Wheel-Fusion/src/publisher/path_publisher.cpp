#include "publisher/path_publisher.hpp"

#include <Eigen/Dense>

PathPublisher::PathPublisher(ros::NodeHandle &nh,
                             std::string topic_name,
                             std::string frame_id,
                             int buff_size)
        : nh_(nh), frame_id_(frame_id) {
    mPath.poses.clear();
    index_ = 0;
    publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
}

void PathPublisher::Reset() {
    mPath.poses.clear();
}

void PathPublisher::Publish(PoseData &pose) {

    Publish(pose.pose, pose.endTime);

}

void PathPublisher::Publish(const Eigen::Matrix4f &pose, double time) {

    mPath.header.stamp = ros::Time::now();
    mPath.header.frame_id = frame_id_;

    geometry_msgs::PoseStamped pose_stamped;
    ros::Time ros_time(time);
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id_;

    pose_stamped.header.seq = index_;

    pose_stamped.pose.position.x = pose(0, 3);
    pose_stamped.pose.position.y = pose(1, 3);
    pose_stamped.pose.position.z = pose(2, 3);

    Eigen::Quaternionf q;
    q = pose.block<3, 3>(0, 0);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    mPath.poses.push_back(pose_stamped);
    index_++;
    publisher_.publish(mPath);
}

void PathPublisher::Publish(const Eigen::Matrix4d &pose, double time) {

    mPath.header.stamp = ros::Time::now();
    mPath.header.frame_id = frame_id_;

    geometry_msgs::PoseStamped pose_stamped;
    ros::Time ros_time(time);
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id_;

    pose_stamped.header.seq = index_;

    pose_stamped.pose.position.x = pose(0, 3);
    pose_stamped.pose.position.y = pose(1, 3);
    pose_stamped.pose.position.z = pose(2, 3);

    Eigen::Quaterniond q;
    q = pose.block<3, 3>(0, 0);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    mPath.poses.push_back(pose_stamped);
    index_++;
    publisher_.publish(mPath);
}

bool PathPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}