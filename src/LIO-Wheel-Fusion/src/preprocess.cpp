#include "preprocess.h"
Preprocess::Preprocess()
        :blind(0.01), max_blind(50), ground_height(0.0), point_filter_num(1) {
    ground_filter_num = 10;
    SCAN_RATE = 10;
    ground_point_num = 0;
    all_point_num = 0;
    jump_up_limit = 170.0;
    jump_down_limit = 8.0;
    cos160 = 160.0;
    smallp_intersect = 172.5;
    jump_up_limit = cos(jump_up_limit / 180 * M_PI);
    jump_down_limit = cos(jump_down_limit / 180 * M_PI);
    cos160 = cos(cos160 / 180 * M_PI);
    smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(double bld, int pfilt_num) {
    blind = bld;
    point_filter_num = pfilt_num;
}


void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out) {

    zvision_handler(msg);
    *pcl_out = pl_surf;
}

void Preprocess::zvision_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    PointCloudXYZI pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();

    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    for (int i = 0; i < plsize; i++) {

        PointType added_pt;

        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].normal_x;  // curvature unit: ms


        /**** 较近距离，以及较远距离点云的去除  ***/

        if(added_pt.z > top_height){
            continue;
        }

        double distance_p = added_pt.x * added_pt.x + added_pt.y * added_pt.y;

        if (distance_p > (max_blind)) {
            continue;
        }

        if (distance_p < (blind)) {
            continue;
        }

        all_point_num += 1;
        /**** 点云降采样  地面点采用更大的降采样倍率 且距离近的地面点用更大的降采样倍率 ***/
        if (added_pt.z < ground_height) {
            ground_point_num += 1;
            if ( (i / 8) % ground_filter_num == 0) {
                pl_surf.points.push_back(added_pt);
            }
        } else {
            if ( (i / 8) % point_filter_num == 0) {  //  8个点为一个扫描周期
                pl_surf.points.push_back(added_pt);
            }
        }
    }
}

void Preprocess::clean_size_buffer() {
    ground_point_num = 0;
    all_point_num = 0;
}
