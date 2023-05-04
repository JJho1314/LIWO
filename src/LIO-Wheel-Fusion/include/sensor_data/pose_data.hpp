#ifndef SRC_SENSOR_DATA_POSE_DATA_HPP_
#define SRC_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

class PoseData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseData() {}

    PoseData(const Eigen::Matrix<float, 4, 4> &pose_, double start, double end) {
        pose = pose_;
        startTime = start;
        endTime = end;
        time = startTime + endTime;
        time = time / 2.0;
    }

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    double time = 0.0;
    double startTime = 0.0;
    double endTime = 0.0;
public:
    Eigen::Quaternionf GetQuaternion();
};

#endif