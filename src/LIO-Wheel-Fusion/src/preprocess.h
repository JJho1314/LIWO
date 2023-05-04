#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class Preprocess {
public:

    Preprocess();

    ~Preprocess();

    void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

    void set(double bld, int pfilt_num);
    void clean_size_buffer();
    PointCloudXYZI pl_full, pl_corn, pl_surf;
    int point_filter_num, SCAN_RATE, ground_filter_num;
    double blind, max_blind, ground_height, top_height, ground_point_num, all_point_num;

private:

    void zvision_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    double jump_up_limit, jump_down_limit;
    double cos160;
    double smallp_intersect;
};
