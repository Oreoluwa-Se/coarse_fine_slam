#ifndef LIDAR_TYPE_STRUCTS_HPP
#define LIDAR_TYPE_STRUCTS_HPP

#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include <Eigen/Dense>

namespace robosense_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D; // This adds the members x, y, z which can also be accessed using the point (which is float[4])
        std::uint8_t intensity;
        double timestamp;
        std::uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

POINT_CLOUD_REGISTER_POINT_STRUCT(
    robosense_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

#endif