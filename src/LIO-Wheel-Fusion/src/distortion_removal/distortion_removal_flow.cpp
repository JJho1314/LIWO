//
// Created by warren on 2022/1/4.
//

//class
#include "distortion_removal/distortion_removal_flow.hpp"
// tools
#include "glog/logging.h"
#include "CommonFunc.h"
#include "config.h"
#include "tools/se3_util.hpp"
#include "iostream"

using namespace std;

DistortRemovalFlow::DistortRemovalFlow(ros::NodeHandle &nh) {
    // Subscriber
    dr_sub_ptr_ = std::make_shared<PoseSourceSubscriber>(nh, Config::dr_odo_topic, 10000);
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, Config::lid_topic, 10000, true);
    // Publisher
    distort_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, Config::distort_cloud_topic, "/camera_init", 10000);
    undistort_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, Config::undistort_cloud_topic, "/camera_init",
                                                                10000);
}

void DistortRemovalFlow::Run() {
    ReadData();

    while (!cloud_data_buffer_.empty()) {
        // the following will always pop 1 cloud data from cloud_data_buffer_ in getCurCloud
        getCurCloud();
        updatePred();
        removeDistortion();
        PublishData();
    }
}

void DistortRemovalFlow::ReadData() {
//        LOG(INFO) << "in ReadData";
    dr_sub_ptr_->ParseData(raw_dr_buffer_);
    cloud_sub_ptr_->ParseData(cloud_data_buffer_);
}

void DistortRemovalFlow::getCurCloud() {
//        LOG(INFO) << "in getCurCloud";
    while (cloud_data_buffer_.size() > 1)
        cloud_data_buffer_.pop_front(); // only process the latest cloud
    current_cloud_data_ = cloud_data_buffer_.front();
    cloud_data_buffer_.pop_front();
}

void DistortRemovalFlow::updatePred() {
//        LOG(INFO) << "in updatePred";
    T_pred.setIdentity();
    while (!raw_dr_buffer_.empty()) {
        // hesai和zvision都是时间戳为t时扫描时间为[t-Config::time_per_lidar(fire time), t]
        customized_msgs::pose_source pose_data = raw_dr_buffer_.front();
        double startTime = pose_data.t0.toSec();
        double endTime = pose_data.t1.toSec();
        if (endTime <= current_cloud_data_.time - Config::time_per_lidar)
            raw_dr_buffer_.pop_front();
        else if (startTime < current_cloud_data_.time) {
            dr_pred_buffer_.push_back(pose_data);
            T_pred = T_pred * GeomPose2EigenMatrixf(pose_data.pose);
            if (endTime <= current_cloud_data_.time)
                raw_dr_buffer_.pop_front();
            else
                break;
        } else
            break;
    }

    if (!dr_pred_buffer_.empty()) { // if empty, T_pred = identity
        canRemoveDistort = true;
        float alpha = Config::time_per_lidar / (dr_pred_buffer_.back().t1.toSec() -
                                                dr_pred_buffer_.front().t0.toSec());
        T_pred = Config::imu_to_lidar_ * scaleSE3(T_pred, alpha) *
                 Config::lidar_to_imu_; // T_prevIMU_curIMU ---> T_prevLiDAR_curLiDAR
        Eigen::Vector3f euler_angles = T_pred.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
//        LOG(INFO) << "#imu used for pred: " << dr_pred_buffer_.size() << ", translxyz: " << T_pred(0, 3) << ", "
//                  << T_pred(1, 3) << ", " << T_pred(2, 3) << ", "
//                  << "rotxyz: " << euler_angles(0) << ", " << euler_angles(1) << ", " << euler_angles(2) << ", "
//                  << ", alpha in dr_pred = " << alpha;
    } else
        canRemoveDistort = false;
//    LOG(INFO) << "can remove distort: " << canRemoveDistort;

    dr_pred_buffer_.clear();
}

void DistortRemovalFlow::removeDistortion() { // change all points to the ending lidar coordinate
    // zvision 与 hesai 本来时间戳为t s就是最后一个点的录制时间，不用修改
    undistort_cloud_data_.time = current_cloud_data_.time;
    undistort_cloud_data_.cloud_ptr->header.stamp = current_cloud_data_.cloud_ptr->header.stamp;
    undistort_cloud_data_.cloud_ptr->points.clear();
    if (canRemoveDistort) {
        int size = current_cloud_data_.cloud_ptr->points.size();
        int pointIndex = 0;
        for (auto iter: current_cloud_data_.cloud_ptr->points) { //将所有点变换至最后一个点录制时的雷达坐标系
            pointIndex++;
            float alpha = 1 - (float)pointIndex / (float)size;
            // T_end_t = T_t_end^-1 = (scaleSE3(T_start_end, alpha))^-1
            Eigen::Matrix4f T_end_t = scaleSE3(T_pred, alpha).inverse();
            Eigen::Vector4f position(iter.x, iter.y, iter.z, 1);
            position = T_end_t * position;
            iter.x = position.x();
            iter.y = position.y();
            iter.z = position.z();
            undistort_cloud_data_.cloud_ptr->points.push_back(iter);
        }
    } else
        undistort_cloud_data_.cloud_ptr->points = current_cloud_data_.cloud_ptr->points;

    if (Config::check_undistort) {
        count++;
        if (Config::save_cloud) {
            if (count >= Config::startSaving_frameCnt && count < Config::startSaving_frameCnt + 20)
                saveCloud();
        }
    }
}

void DistortRemovalFlow::saveCloud() {
    LOG(INFO) << "saving cloud " << count - Config::startSaving_frameCnt;
    string save_path = WORK_SPACE_PATH + "/distortion_removal_check/";
    string distort_path = save_path + "distort" + to_string(count - Config::startSaving_frameCnt) + ".pcd";
    string undistort_path = save_path + "undistort" + to_string(count - Config::startSaving_frameCnt) + ".pcd";
    current_cloud_data_.cloud_ptr->height = 1;
    current_cloud_data_.cloud_ptr->width = current_cloud_data_.cloud_ptr->size();
    undistort_cloud_data_.cloud_ptr->height = 1;
    undistort_cloud_data_.cloud_ptr->width = undistort_cloud_data_.cloud_ptr->size();
    pcl::io::savePCDFileASCII(distort_path, *current_cloud_data_.cloud_ptr);
    pcl::io::savePCDFileASCII(undistort_path, *undistort_cloud_data_.cloud_ptr);
}

void DistortRemovalFlow::PublishData() {
//        LOG(INFO) << "in PublishData";
    // 时间戳已在removDistortion()里改为最后一个点的录制时间
    distort_cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    undistort_cloud_pub_ptr_->Publish(undistort_cloud_data_.cloud_ptr, undistort_cloud_data_.time);
}