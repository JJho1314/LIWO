
#include "evo_apriltag/evo_apriltag.h"
#include "config.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/PinholeCamera.h"

deque<pair<double, cv::Mat>> AprilTag_buf_start, AprilTag_buf_latest;

const int AprilTagSize = 9;

/**
 * @brief 将开始原始图像帧保存到buf中，只保留开始和结尾的图像
 */
void AprilTag_buf_push(const double &time, const cv::Mat &image, bool init)
{
    if(init)///estimator.solver_flag == SolverFlag::INITIAL
    {
        while(AprilTag_buf_start.size() >= AprilTagSize)
            AprilTag_buf_start.pop_front();
        AprilTag_buf_start.emplace_back(make_pair(time, image));
    } else
    {
        while(AprilTag_buf_latest.size() >= AprilTagSize)
            AprilTag_buf_latest.pop_front();
        AprilTag_buf_latest.emplace_back(make_pair(time+Config::td, image));
    }
}

/**
 * @brief 当初始化完成之后将VIO轨迹第一帧之前保存在buf里面的图像过滤掉，并将剩余buf的后一部分删掉，只保留3帧
 * @param first_time VIO轨迹第一帧的时间
 */
void AprilTag_buf_filter(const double &first_time)
{
    cerr << "time of latest item in start buf: " << AprilTag_buf_start.back().first;
    cerr << "time of the beginning fused pose: " << first_time;
    while (AprilTag_buf_start.front().first < first_time - 0.001 && AprilTag_buf_start.size() > 0)
        AprilTag_buf_start.pop_front();
    while (AprilTag_buf_start.size() > AprilTagSize)
        AprilTag_buf_start.pop_back();
}

/**
 * @brief 利用buf里面的AprilTag图像和估计的轨迹进行轨迹评估
 */
bool EVO(const double &total_length, deque<std::pair<double, Eigen::Matrix4d>> &FusedPath_start, deque<std::pair<double, Eigen::Matrix4d>> &FusedPath_latest)///for apriltag)
{

    std::string camPath = WORK_SPACE_PATH + "/config/" + Config::cam_param;
    camodocal::CameraPtr m_camera;
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(camPath);
    cv::Mat image;
    std_msgs::Header header;
    double time_evo = 0, time_est;
    double dt = 1.0/Config::FREQ/2.0-0.001;//according to high FREQ
    Eigen::Matrix4d evo_Twc_start = Eigen::Matrix4d::Identity(),
                    evo_Twc_latest = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d est_Tbw_start = Eigen::Matrix4d::Identity(),
                    est_Tbw_latest = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d evo_relative, est_relative;
    bool start_isCalOK = false, latest_isCalOK = false;

    vector<pair<Eigen::Matrix4d, Eigen::Matrix4d>> evo_est_poses_start, evo_est_poses_latest;
    while (!AprilTag_buf_start.empty() && !FusedPath_start.empty()){
        time_evo = AprilTag_buf_start.front().first;
        time_est = FusedPath_start.front().first;//low frequency
        if (time_est - time_evo < dt){ //abs
            start_isCalOK = calcCamPose(time_evo,
                                        AprilTag_buf_start.front().second,
                                        m_camera,
                                        evo_Twc_start);
            if (start_isCalOK){
                est_Tbw_start = FusedPath_start.front().second;
                evo_est_poses_start.emplace_back(make_pair(evo_Twc_start, est_Tbw_start));
            }
            FusedPath_start.pop_front();
            AprilTag_buf_start.pop_front();
        }
        else
            AprilTag_buf_start.pop_front();//high FREQ
    }
    while (!AprilTag_buf_latest.empty()&& !FusedPath_latest.empty()){
        time_evo = AprilTag_buf_latest.back().first;
        time_est = FusedPath_latest.back().first;//low frequency
        if (time_evo - time_est < dt){ //不要abs 防止最后一个图像帧早于位姿帧
            start_isCalOK = calcCamPose(time_evo,
                                        AprilTag_buf_latest.back().second,
                                        m_camera,
                                        evo_Twc_latest);
            if (start_isCalOK) {
                est_Tbw_latest = FusedPath_latest.back().second;
                evo_est_poses_latest.emplace_back(make_pair(evo_Twc_latest, est_Tbw_latest));
            }
            FusedPath_latest.pop_back();
            AprilTag_buf_latest.pop_back();
        }
        else
            AprilTag_buf_latest.pop_back();//high FREQ
    }

    if(evo_est_poses_start.empty() | evo_est_poses_latest.empty()) return false;

    Eigen::Matrix4d body_T_cam0 = Eigen::Matrix4d::Identity();
    body_T_cam0.block<3, 3>(0, 0) = Config::ric;
    body_T_cam0.block<3, 1>(0, 3) = Config::tic;

    Eigen::Matrix4d Error = Eigen::Matrix4d::Identity();
    vector<Eigen::Matrix4d> Error_all;
    for(auto & it1 : evo_est_poses_start)
    {
        for(auto & it2 : evo_est_poses_latest)
        {
            evo_relative = (body_T_cam0 * it1.first) *(body_T_cam0 * it2.first).inverse();
            est_relative = it1.second.inverse() * it2.second;
            Error = evo_relative.inverse() * est_relative;
            Error_all.emplace_back(Error);
        }
    }
    for(auto & it : Error_all)
    {
        Eigen::Vector3d Error_rot = R2ypr(Error.block<3, 3>(0,0));
        Eigen::Vector3d Error_rot_i = R2ypr(it.block<3, 3>(0,0));
        if(Error_rot_i.norm() < Error_rot.norm())
            Error = it;
    }

    Eigen::Vector3d Error_rot = R2ypr(Error.block<3, 3>(0,0)); // Yaw-Pitch-Roll  (deg)
    Eigen::AngleAxisd Error_Angle(Error.block<3, 3>(0,0));
    Eigen::Vector3d Error_tran(Error.block<3,1>(0,3));
    return true;
}

///TODO ZHX:直接利用本身有的R2euler
static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}
