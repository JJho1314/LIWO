//
// Created by warren on 2021/12/28.
//
#include <ros/ros.h>
#include "glog/logging.h"
#include "tools/file_manager.hpp"
#include "global_definition/global_definition.h"
#include "pose_source/dr_odo_flow.hpp"
#include "config.h"

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "dr_odo_node");
    ros::NodeHandle nh("~");

    bool screen = false;
    nh.param<bool>("screen", screen, "true");

    google::InitGoogleLogging(argv[0]);
    if (!boost::filesystem::is_directory(WORK_SPACE_PATH + "/Log")) {
        boost::filesystem::create_directory(WORK_SPACE_PATH + "/Log");
    }
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log/programLog";
    FLAGS_alsologtostderr = screen;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;
    FileManager::CreateDirectory(FLAGS_log_dir);

    Config::readConfig();

    std::shared_ptr<DrOdoFlow> dr_odo_flow_ptr = std::make_shared<DrOdoFlow>(nh);

    ros::Rate rate(1000);

    while (ros::ok()) {
        ros::spinOnce();
        dr_odo_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}