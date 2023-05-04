#ifndef SRCS_SENSOR_DATA_VELOCITY_DATA_HPP_
#define SRCS_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

class VelocityData {
public:
    VelocityData() {
        linear_velocity.x = 0.0;
        linear_velocity.y = 0.0;
        linear_velocity.z = 0.0;
        angular_velocity.x = 0.0;
        angular_velocity.y = 0.0;
        angular_velocity.z = 0.0;
    }

    struct LinearVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

public:
    static bool
    SyncData(std::deque<VelocityData> &UnsyncedData, std::deque<VelocityData> &SyncedData, double sync_time);

    void TransformCoordinate(Eigen::Matrix4d transform_matrix);

    VelocityData operator+(const VelocityData &v) {
        VelocityData vel;
        vel.time = this->time + v.time;
        vel.linear_velocity.x = this->linear_velocity.x + v.linear_velocity.x;
        vel.linear_velocity.y = this->linear_velocity.y + v.linear_velocity.y;
        vel.linear_velocity.z = this->linear_velocity.z + v.linear_velocity.z;
        vel.angular_velocity.x = this->angular_velocity.x + v.angular_velocity.x;
        vel.angular_velocity.y = this->angular_velocity.y + v.angular_velocity.y;
        vel.angular_velocity.z = this->angular_velocity.z + v.angular_velocity.z;
        return vel;
    }

    template<typename T>
    VelocityData operator/(const T &cnt) {
        VelocityData vel;
        vel.time = this->time / cnt;
        vel.linear_velocity.x = this->linear_velocity.x / cnt;
        vel.linear_velocity.y = this->linear_velocity.y / cnt;
        vel.linear_velocity.z = this->linear_velocity.z / cnt;
        vel.angular_velocity.x = this->angular_velocity.x / cnt;
        vel.angular_velocity.y = this->angular_velocity.y / cnt;
        vel.angular_velocity.z = this->angular_velocity.z / cnt;
        return vel;
    }

    template<typename T>
    VelocityData operator*(const T &cnt) {
        VelocityData vel;
        vel.time = this->time * cnt;
        vel.linear_velocity.x = this->linear_velocity.x * cnt;
        vel.linear_velocity.y = this->linear_velocity.y * cnt;
        vel.linear_velocity.z = this->linear_velocity.z * cnt;
        vel.angular_velocity.x = this->angular_velocity.x * cnt;
        vel.angular_velocity.y = this->angular_velocity.y * cnt;
        vel.angular_velocity.z = this->angular_velocity.z * cnt;
        return vel;
    }

    Eigen::Vector3d as_vector() {
        return Eigen::Vector3d(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    }

};

#endif