#pragma once

#include <pcl/point_types.h>

namespace rslidar_pointcloud
{

// Used for calibration diagnostics
// this struct is copied from drive/common/include/pcl/pcl_types.h
struct PointXYZIRT {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
        PCL_ADD_POINT4D;             // quad-word XYZ
    float intensity;                 ///< laser intensity reading
    uint16_t ring;                   ///< laser ring number
    double timestamp;                ///< laser timestamp

} EIGEN_ALIGN16;

}  // namespace rslidar_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_pointcloud::PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (double, timestamp, timestamp));

