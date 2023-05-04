//
// Created by warren on 2022/1/4.
//

#include <ros/ros.h>
#include "glog/logging.h"
#include <boost/filesystem.hpp>
#include "tools/file_manager.hpp"
#include "global_definition/global_definition.h"
#include "config.h"
#include "distortion_removal/distortion_removal_flow.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "distortion_removal_node");
    ros::NodeHandle nh("~");

    bool screen = false;
    nh.param<bool>("screen", screen, "true");

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log/programLog";
    FLAGS_alsologtostderr = screen;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;
    FileManager::CreateDirectory(FLAGS_log_dir);

    LOG(INFO) << "distortion_removal/screen: " << screen;

    Config::readConfig();

    std::shared_ptr<DistortRemovalFlow> distort_removal_flow_ptr = std::make_shared<DistortRemovalFlow>(nh);

    ros::Rate rate(1000);
    while (ros::ok()) {
        ros::spinOnce();
        distort_removal_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}