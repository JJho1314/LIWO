/*
 * @Description: 订阅cam数据
 */
#include "subscriber/cam_subscriber.hpp"
#include "glog/logging.h"
#include "evo_apriltag/evo_apriltag.h"

CamSubscriber::CamSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CamSubscriber::msg_callback, this);
}
void CamSubscriber::msg_callback(const sensor_msgs::ImageConstPtr &cam_msg_ptr) {
    buff_mutex_.lock();
    if (new_cam_data_.size() > 7)
        new_cam_data_.pop_front();
    new_cam_data_.push_back(cam_msg_ptr);
    buff_mutex_.unlock();
}

void CamSubscriber::ParseData(bool has_init) {
    buff_mutex_.lock();
    while (!new_cam_data_.empty()){
        double time;
        cv::Mat image;
        time = new_cam_data_.front()->header.stamp.toSec();
        image = getImageFromMsg(new_cam_data_.front());
        AprilTag_buf_push(time, image, has_init);
        new_cam_data_.pop_front();
    }
    buff_mutex_.unlock();
}

cv::Mat CamSubscriber::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}
