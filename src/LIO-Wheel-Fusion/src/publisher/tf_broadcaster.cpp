#include "publisher/tf_broadcaster.hpp"

TFBroadCaster::TFBroadCaster(std::string frame_id, std::string child_frame_id) {
    transform_.frame_id_ = frame_id;
    transform_.child_frame_id_ = child_frame_id;
}

void TFBroadCaster::SendTransform(Eigen::Matrix4f pose, double time) {
    Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
    ros::Time ros_time(time);
    transform_.stamp_ = ros_time;
    transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()).normalize());
    transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
    broadcaster_.sendTransform(transform_);
}

void TFBroadCaster::SendTransform(Eigen::Matrix4d pose, double time) {
    Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    ros::Time ros_time(time);
    transform_.stamp_ = ros_time;
    transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()).normalize());
    transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
    broadcaster_.sendTransform(transform_);
}