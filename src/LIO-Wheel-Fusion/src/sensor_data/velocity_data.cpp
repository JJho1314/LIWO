#include "sensor_data/velocity_data.hpp"

#include "glog/logging.h"

bool VelocityData::SyncData(std::deque<VelocityData> &UnsyncedData, std::deque<VelocityData> &SyncedData,
                            double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    bool diff_large = false;
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
//            允许两帧需要同步的数据都在雷达之前
        if (UnsyncedData.at(1).time < sync_time && UnsyncedData.size() > 2) {
            UnsyncedData.pop_front();
            continue;
        }
//            fixme 频率
        if (sync_time - UnsyncedData.front().time > 0.5) {
            UnsyncedData.pop_front();
            diff_large = true;
            break;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.5) {
            UnsyncedData.pop_front();
            diff_large = true;
            break;
        }
        break;
    }

    VelocityData synced_data;
    if (UnsyncedData.size() < 2) {
//            因为轮速计代码设计，长时间不返数，默认轮速为0
        if (diff_large) {
            synced_data.time = sync_time;
            SyncedData.push_back(synced_data);
            return true;
        }
        return false;
    }

    VelocityData front_data = UnsyncedData.at(0);
    VelocityData back_data = UnsyncedData.at(1);
    if (back_data.time - front_data.time == 0.0) {
        UnsyncedData.pop_front();
        return false;
    }

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_velocity.x =
            front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
    synced_data.linear_velocity.y =
            front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
    synced_data.linear_velocity.z =
            front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
    synced_data.angular_velocity.x =
            front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y =
            front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z =
            front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

//        LOG(INFO) << "front_scale: " << front_scale << "\tback_scale: " << back_scale;
//        LOG(INFO) << "SyncedData: " << SyncedData.size() << " back_data: " << synced_data.linear_velocity.x << "\t" << synced_data.linear_velocity.y << "\t" << synced_data.linear_velocity.z;
    SyncedData.push_back(synced_data);

    return true;
}

//    输入T_v^i
void VelocityData::TransformCoordinate(Eigen::Matrix4d transform_matrix) {
    Eigen::Matrix4d matrix = transform_matrix.cast<double>();
    Eigen::Matrix3d t_R = matrix.block<3, 3>(0, 0);
    Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
    Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    //w_i = R_v^i * w_v
    //v_i = R_v^i * w_v x t_v^i + R_v^i * v_v
    w = t_R * w;
    v = t_R * v;
    Eigen::Vector3d r(matrix(0, 3), matrix(1, 3), matrix(2, 3));
    Eigen::Vector3d delta_v;
    delta_v(0) = w(1) * r(2) - w(2) * r(1);
    delta_v(1) = w(2) * r(0) - w(0) * r(2);
    delta_v(2) = w(0) * r(1) - w(1) * r(0);
    v = v + delta_v;

    angular_velocity.x = w(0);
    angular_velocity.y = w(1);
    angular_velocity.z = w(2);
    linear_velocity.x = v(0);
    linear_velocity.y = v(1);
    linear_velocity.z = v(2);
}