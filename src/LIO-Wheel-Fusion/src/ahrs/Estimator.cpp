#include <ahrs/Estimator.h>
#include "config.h"
#include <iostream>

namespace OriEst {

    IMUData preIntegrate(const IMUData &imu_in, const std::shared_ptr<OriEst::Estimator> &orientation_estimator) {

        IMUData imuSelf = imu_in;
        Eigen::Matrix3d G_R_I;
        const double timestamp = imu_in.time;
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        Eigen::Vector3d gyro(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);

//    Eigen::Matrix3d R = Config::car_to_imu_.block<3,3>(0,0).cast<double>(); // 旧车IMU很离谱地发在car坐标系下所以才会乘car_to_imu_的坐标变换，新车不用
//#ifdef SJTU
//    gyro = R * gyro;
//                acc = R * acc;
//#endif
        imuSelf.angular_velocity.x = gyro(0);
        imuSelf.angular_velocity.y = gyro(1);
        imuSelf.angular_velocity.z = gyro(2);
        imuSelf.linear_acceleration.x = acc(0);
        imuSelf.linear_acceleration.y = acc(1);
        imuSelf.linear_acceleration.z = acc(2);
        OriEst::Status status = orientation_estimator->Estimate(timestamp, gyro, acc, &G_R_I);
        Eigen::Quaterniond quat(G_R_I);
        imuSelf.orientation.Construct(quat);

        return imuSelf;
    }


    Estimator::Estimator(const double gyro_noise, const double gyro_bias_noise, const double acc_noise)
            : status_(Status::kInvalid),
              last_timestamp_(-1.),
              initializer_(std::make_unique<Initializer>()),
              propagator_(std::make_unique<Propagator>(gyro_noise, gyro_bias_noise)) {
        acc_noise_mat_ = Eigen::Matrix3d::Identity() * acc_noise;
    }

    Status Estimator::Estimate(double timestamp, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc,
                               Eigen::Matrix3d *G_R_I) {
        if (status_ == Status::kInvalid) {
            if (!initializer_->Initialize(acc, &G_R_I_)) {
                G_R_I->setIdentity();
                return Status::kInvalid;
            }

            bg_.setZero();
            cov_.setZero();
            cov_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.5 * 0.5 * kDeg2Rad * kDeg2Rad;
            cov_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.1 * 0.1 * kDeg2Rad * kDeg2Rad;
            last_timestamp_ = timestamp;

            // Send out.
            *G_R_I = G_R_I_;

            status_ = Status::kValid;
            return Status::kValid;
        }

        double delta_t = timestamp - last_timestamp_;
        last_timestamp_ = timestamp;

        // Propagation.
        Eigen::Matrix3d prior_G_R_I;
        Eigen::Vector3d prior_bg;
        Eigen::Matrix<double, 6, 6> prior_cov;
        propagator_->PropagateMeanAndCov(G_R_I_, bg_, cov_, gyro, delta_t, &prior_G_R_I, &prior_bg, &prior_cov);

        acc_buffer_.push_back(acc);
        if (acc_buffer_.size() > config_.acc_buffer_size) {
            acc_buffer_.pop_front();
        }
        // compute mean.
        Eigen::Vector3d mean_acc(0., 0., 0.);
        for (const Eigen::Vector3d &one_acc: acc_buffer_) {
            mean_acc += one_acc;
        }
        mean_acc = mean_acc / static_cast<double>(acc_buffer_.size());

        // Update
        Update(prior_G_R_I, prior_bg, prior_cov, mean_acc, acc_noise_mat_, &G_R_I_, &bg_, &cov_);

        // Send out.
        *G_R_I = G_R_I_;

        return Status::kValid;
    }

}  // namespace OriEst