#ifndef VISUAL_HELPERS_HPP
#define VISUAL_HELPERS_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <map_storage/utils/alias.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <deque>

template <typename T>
struct VisualFuncs
{
    /*
     * Modified from: https://github.com/PRBonn/kiss-icp/
     */

    static geometry_msgs::Transform sophus_to_transform(const SE3Type<T> &pose);

    static geometry_msgs::Pose sophus_to_pose(const SE3Type<T> &pose);

    static SE3Type<T> transform_to_sophus(const geometry_msgs::TransformStamped &transform);

    static std::unique_ptr<sensor_msgs::PointCloud2> eig_to_pc_2(const std::deque<Point3dWPtr<T>> &points, size_t size, const std_msgs::Header &header);

    static std::unique_ptr<sensor_msgs::PointCloud2> eig_to_pc_2(const Point3dWPtrVecCC<T> &points, const std_msgs::Header &header);

    static std::unique_ptr<sensor_msgs::PointCloud2> eig_to_pc_2(const std::deque<Eigen::Matrix<T, 3, 1>> &points, size_t size, const std_msgs::Header &header);

    static void fill_point_cloud(const Point3dWPtrVecCC<T> &points, sensor_msgs::PointCloud2 &msg);

    static void fill_point_cloud(const std::deque<Eigen::Matrix<T, 3, 1>> &points, sensor_msgs::PointCloud2 &msg);

    static void fill_point_cloud(const std::deque<Point3dWPtr<T>> &points, sensor_msgs::PointCloud2 &msg);

    static std::unique_ptr<sensor_msgs::PointCloud2> create_pc2_msg(const size_t n_points, const std_msgs::Header &header, bool timestamp = false);
};

#endif