#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "coarse_fine_slam/helpers/visual_helper.hpp"
#include "coarse_fine_slam/state_estimation/interface.hpp"
#include "parameters.hpp"
#include <map_storage/sections/tree_manager.hpp>
#include <memory>
#include <deque>

// ros inclusions
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

template <typename T>
class Visualizer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<Visualizer<T>>;

    Visualizer(ros::NodeHandle &nh);

    void new_data(EndofCycleResults<T> &data, bool add_to_nav);

    void set_id_info(const std::string imu_header, const std::string pc_header);

    SE3Type<T> lookup_transform(const std::string &target_frame, const std::string &source_frame);

private:
    void publish_odom(EndofCycleResults<T> &info, const ros::Time &stamp, bool add_to_nav = false);

    void publish_cloud(EndofCycleResults<T> &data, const ros::Time &stamp);

private:
    /* ..................... Ros attributes .....................  */
    ros::NodeHandle nh;

    // Tools for Broadcasting
    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    // Publishers
    ros::Publisher odom_publisher;
    ros::Publisher map_publisher;
    ros::Publisher curr_frame_publisher;
    ros::Publisher traj_publisher;
    nav_msgs::Path path_msg;

    int queue_size{1};

    std::deque<Eigen::Matrix<T, 3, 1>> frames;
    // We basically store a track of subframes and visualize that
    std::deque<size_t> frame_sizes;

    /* .....................  Other attributes ..................... */
    std::string pc_header_frame_id = "", imu_header_frame_id = "", moving_frame = "";
    bool ego_est = false;
};

template <typename T>
using VisualizerPtr = typename Visualizer<T>::Ptr;
#endif