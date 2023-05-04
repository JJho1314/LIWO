#include "pose_source/dr_odo_flow.hpp"
#include "global_definition/global_definition.h"
#include "CommonFunc.h"
#include "config.h"
#include "tools/se3_util.hpp"

using namespace std;

DrOdoFlow::DrOdoFlow(ros::NodeHandle &nh) : Twi(Eigen::Matrix4d::Identity()) {
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, Config::imu_topic, 100000);
    vel_sub_ptr_ = std::make_shared<OdomSubscriber>(nh, Config::vel_topic, 100000, true);
    sjtu_vel_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, Config::sjtu_vel_topic, 100000, true);
    dr_odo_pub_ptr_ = std::make_shared<PoseSourcePublisher>(nh, Config::dr_odo_topic, 100000);
    // for rviz
    tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", Config::dr_odo_tf_topic);
    path_pub_ptr_ = std::make_shared<PathPublisher>(nh, Config::dr_odo_path_topic, "/map", 100000);
    // Orientation Estimator
    const double gyro_noise = 1e-6;  // default  1e-6
    const double gyro_bias_noise = 1e-8; // default 1e-8
    const double acc_noise = 1e-2;  // default 1e-1
    orientation_estimator = std::make_shared<OriEst::Estimator>(gyro_noise, gyro_bias_noise, acc_noise);
}

void DrOdoFlow::Run() {
    ReadData();
    while (!raw_imu_.empty()) {
        updateDrOdo();
        if (!dr_buffer.empty())
            PublishData();
    }
}

void DrOdoFlow::ReadData() {
    imu_sub_ptr_->ParseData(raw_imu_);
    if(Config::use_sjtu_config){
        sjtu_vel_sub_ptr_->ParseData(unsynced_velocity_);
    }
    else{
        vel_sub_ptr_->ParseData(unsynced_velocity_);
    }
}

void DrOdoFlow::updateDrOdo() {
    while (!raw_imu_.empty()) {
        IMUData imu_in = raw_imu_.front();
        IMUData imuSelf = OriEst::preIntegrate(imu_in, orientation_estimator);
        unsynced_imu_.push_back(imuSelf);
        raw_imu_.pop_front();
    }

    while (unsynced_imu_.size() > 1) {
        IMUData imu_front = unsynced_imu_.at(0);
        IMUData imu_back = unsynced_imu_.at(1);
        if (imu_front.time == imu_back.time) {
            unsynced_imu_.pop_front();
            continue;
        }

        Eigen::Matrix4d T_prev_cur_imu = Fusing_IMU_VelOdom(imu_front, imu_back);

        unsynced_imu_.pop_front();

        customized_msgs::pose_source dr_temp;
        dr_temp.isBinary = true;
        dr_temp.t0 = ros::Time(imu_front.time);
        dr_temp.t1 = ros::Time(imu_back.time);
        dr_temp.pose.position.x = T_prev_cur_imu(0, 3);
        dr_temp.pose.position.y = T_prev_cur_imu(1, 3);
        dr_temp.pose.position.z = T_prev_cur_imu(2, 3);
        Eigen::Quaterniond q = Eigen::Quaterniond(T_prev_cur_imu.block<3, 3>(0, 0));
        dr_temp.pose.orientation.x = q.x();
        dr_temp.pose.orientation.y = q.y();
        dr_temp.pose.orientation.z = q.z();
        dr_temp.pose.orientation.w = q.w();
        dr_buffer.push_back(dr_temp);
        Twi = Twi * T_prev_cur_imu;
    }
}

Eigen::Matrix4d DrOdoFlow::Fusing_IMU_VelOdom(IMUData imu_front, IMUData imu_back) {

    Eigen::Matrix4d T_prev_cur_imu = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d J_prev_cur_imu = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_prev_cur_imu = Eigen::Matrix3d::Identity();

    R_prev_cur_imu = imu_front.orientation.getRoation().transpose() * imu_back.orientation.getRoation();
    Eigen::AngleAxisd delta_se3(R_prev_cur_imu);
    double phi = delta_se3.angle();
    Eigen::Vector3d axis = delta_se3.axis();
    if (abs(phi) < 0.00001)
        J_prev_cur_imu = Eigen::Matrix3d::Identity();
    else
        J_prev_cur_imu = sin(phi) / phi * Eigen::Matrix3d::Identity() + (1 - sin(phi) / phi) * axis * axis.transpose() +
                         ((1 - cos(phi)) / phi) * skew(axis);

    Eigen::Vector3d ang_vel(imu_back.angular_velocity.x, imu_back.angular_velocity.y, imu_back.angular_velocity.z);
    Eigen::Vector3d u = updateU(unsynced_velocity_, imu_front.time, imu_back.time, ang_vel);
    T_prev_cur_imu.block(0, 0, 3, 3) << R_prev_cur_imu;
    T_prev_cur_imu.block(0, 3, 3, 1) << J_prev_cur_imu * u;
    T_prev_cur_imu.block(3, 0, 1, 4) << 0, 0, 0, 1;

    return T_prev_cur_imu;
}

void DrOdoFlow::WheelVel2IMUVel(VelocityData &vel_data, const Eigen::Vector3d &ang_vel) { // ang_vel measured by IMU, and is expressed in IMU coordinate
    // If all expressed in IMU coordinate, v_IMU = v_Wheel + ang_vel × r_Wheel_IMU
    Eigen::Matrix3d R_IMU_Wheel = Config::wheel_to_imu_.block<3,3>(0,0);
    Eigen::Vector3d r_Wheel_IMU = R_IMU_Wheel * Config::imu_to_wheel_.block<3,1>(0,3); // expressed in IMU coordinate ￥imu_to_wheel_
    Eigen::Vector3d v_Wheel(vel_data.linear_velocity.x, vel_data.linear_velocity.y, vel_data.linear_velocity.z); // expressed in Wheel coordinate
    Eigen::Vector3d v_IMU = R_IMU_Wheel * v_Wheel + ang_vel.cross(r_Wheel_IMU);
    vel_data.linear_velocity.x = v_IMU(0);
    vel_data.linear_velocity.y = v_IMU(1);
    vel_data.linear_velocity.z = v_IMU(2);
}

Eigen::Vector3d DrOdoFlow::updateU(std::deque<VelocityData> &unsynced_velocity, double front_time, double back_time, const Eigen::Vector3d &ang_vel) {

    std::deque<VelocityData> velocities;
    while (!unsynced_velocity.empty()) {
        VelocityData vel = unsynced_velocity.front();
        if (vel.time < front_time)
            unsynced_velocity.pop_front();
        else if (vel.time >= front_time && vel.time <= back_time) {
            unsynced_velocity.pop_front();
            WheelVel2IMUVel(vel, ang_vel);
            velocities.push_back(vel);
        } else
            break;
    }
    VelocityData cur_velocity;
    int size = velocities.size();
    while (!velocities.empty()) {
        cur_velocity = cur_velocity + velocities.front();
        velocities.pop_front();
    }
    cur_velocity = size > 0 ? cur_velocity / size : cur_velocity;

    if(Config::use_sjtu_config){
        if (abs((front_time + back_time) / 2.0 - sjtu_vel_sub_ptr_->nonZeroLast.time) < 0.11 && size == 0){
            cur_velocity = sjtu_vel_sub_ptr_->nonZeroLast;
            WheelVel2IMUVel(cur_velocity, ang_vel);
        }
    }
    else{
        if (abs((front_time + back_time) / 2.0 - vel_sub_ptr_->nonZeroLast.time) < 0.11 && size == 0){
            cur_velocity = vel_sub_ptr_->nonZeroLast;
            WheelVel2IMUVel(cur_velocity, ang_vel);
        }
    }


    VelocityData current_velocity_data_ = cur_velocity;
    Eigen::Vector3d temp;

    temp = (current_velocity_data_ * (back_time - front_time)).as_vector();

    return temp;
}

void DrOdoFlow::PublishData() {
    while (!dr_buffer.empty()) {
        dr_odo_pub_ptr_->PublishData(dr_buffer.front());
        tf_pub_ptr_->SendTransform(Twi, dr_buffer.front().t1.toSec());
        path_pub_ptr_->Publish(Twi, dr_buffer.front().t1.toSec());
        dr_buffer.pop_front();
    }
}
