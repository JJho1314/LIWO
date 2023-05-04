#ifndef SRCS_SENSOR_DATA_IMU_DATA_HPP_
#define SRCS_SENSOR_DATA_IMU_DATA_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include "CommonFunc.h"

class IMUData {
public:
    struct LinearAcceleration {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct RPY {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    class Orientation {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;

    public:
        void Normlize() {
            double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }

        RPY getRPY() {
            Eigen::Quaterniond q(w, x, y, z);
            Eigen::Matrix3d rx = q.toRotationMatrix();
            Eigen::Vector3d ea = rx.eulerAngles(2, 1, 0);
            double roll, pitch, yaw;
            toEulerAngle(q, roll, pitch, yaw);
            RPY rpy;
            rpy.roll = roll;
            rpy.pitch = pitch;
            rpy.yaw = yaw;
            return rpy;
        }

        void Construct(Eigen::Quaterniond &quat) {
            x = quat.x();
            y = quat.y();
            z = quat.z();
            w = quat.w();
            Normlize();
        }

        Eigen::Matrix3d getRoation() {
            Eigen::Quaterniond q(w, x, y, z);
            Eigen::Matrix3d rx = q.toRotationMatrix();
            return rx;
        }
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
//        geometry_msgs::Quaternion_ <std::allocator<void>> orientation;
    Orientation orientation;

public:
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetOrientationMatrix();

    static bool SyncData(std::deque<IMUData> &UnsyncedData, std::deque<IMUData> &SyncedData, double sync_time);

    template<typename T>
    static IMUData transform(const IMUData &imu_in, const Eigen::Matrix<T, 4, 4> &imu_to_other);
};

#endif