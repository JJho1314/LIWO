#ifndef SRC_SENSOR_DATA_CLOUD_DATA_HPP_
#define SRC_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class CloudData {
public:
    using POINT = pcl::PointXYZINormal;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

public:
    CloudData()
            : cloud_ptr(new CLOUD()) {
        cloud_ptr->clear();
    }

public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};

typedef CloudData CloudDataPre; // fixme: 不知道在LIO_Wheel里面用不用得着


#endif