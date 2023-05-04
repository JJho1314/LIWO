/*
 * @Description: 订阅cam数据
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class CamSubscriber {
public:
    CamSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);

    CamSubscriber() = default;

    void ParseData(bool has_init);

private:
    void msg_callback(const sensor_msgs::ImageConstPtr &cam_msg_ptr);

    cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<sensor_msgs::ImageConstPtr> new_cam_data_;

    std::mutex buff_mutex_;
};

#endif