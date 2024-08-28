#include "coarse_fine_slam/ros_1/visualizer.hpp"
#include <boost/array.hpp>

namespace
{
    constexpr size_t online_sub_frame_trail = 50;
}

template <typename T>
Visualizer<T>::Visualizer(ros::NodeHandle &nh)
    : tf2_buffer(), tf2_listener(tf2_buffer)
{
    odom_publisher = nh.advertise<nav_msgs::Odometry>("/cfs/odometry", queue_size);
    traj_publisher = nh.advertise<nav_msgs::Path>("/cfs/trajectory", queue_size);

    curr_frame_publisher = nh.advertise<sensor_msgs::PointCloud2>("/cfs/curr_frame", queue_size);
    map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/cfs/total_map", queue_size);
}

template <typename T>
void Visualizer<T>::set_id_info(const std::string imu_header, std::string pc_header)
{
    // selecting frame headers
    imu_header_frame_id = imu_header;
    pc_header_frame_id = params<T>.imu_en ? imu_header : pc_header; // when we have imu imu frame else lidar frame

    // frame for
    path_msg.header.frame_id = params<T>.odom_frame;

    // pose information
    ego_est = (params<T>.base_frame.empty() || params<T>.base_frame == pc_header_frame_id);
    moving_frame = ego_est ? pc_header_frame_id : params<T>.base_frame;
}

template <typename T>
void Visualizer<T>::new_data(EndofCycleResults<T> &data, bool add_to_nav)
{
    // handle
    auto curr_time_stamp = ros::Time::now();

    // publish odometry
    publish_odom(data, curr_time_stamp, add_to_nav);

    // visualize map
    publish_cloud(data, curr_time_stamp);
}

template <typename T>
void Visualizer<T>::publish_odom(EndofCycleResults<T> &data, const ros::Time &stamp, bool add_to_nav)
{
    if (!ego_est)
    {
        // transform to from ego est to specified base_link/footprint
        SE3Type<T> cloud_to_base = lookup_transform(params<T>.base_frame, pc_header_frame_id);
        data.curr_pose = cloud_to_base * data.curr_pose * cloud_to_base.inverse();
    }

    // broadcast the transform
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = params<T>.odom_frame;
    transform_msg.child_frame_id = moving_frame;
    transform_msg.transform = VisualFuncs<T>::sophus_to_transform(data.curr_pose);
    tf_broadcaster.sendTransform(transform_msg);

    // publish odometry msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = params<T>.odom_frame;
    odom_msg.child_frame_id = moving_frame;

    // populate covariance matrix
    boost::array<T, 36> pose_cov;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            pose_cov[i * 6 + j] = data.pose_cov(i, j);

    odom_msg.pose.covariance = pose_cov;
    odom_msg.pose.pose = VisualFuncs<T>::sophus_to_pose(data.curr_pose);
    odom_publisher.publish(odom_msg);

    // publish trajectory msg
    if (add_to_nav)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = params<T>.odom_frame;
        pose_msg.pose = VisualFuncs<T>::sophus_to_pose(data.curr_pose);
        path_msg.poses.push_back(pose_msg);
        traj_publisher.publish(path_msg);
    }
}

template <typename T>
void Visualizer<T>::publish_cloud(EndofCycleResults<T> &data, const ros::Time &stamp)
{
    // header for current pointcloud printing stuff
    std_msgs::Header c_header;
    c_header.stamp = stamp;
    c_header.frame_id = pc_header_frame_id;

    // populate vector
    if (!data.frame_points.empty())
    {
        // add to stack of points currently being tracked
        frame_sizes.push_back(data.frame_points.size());

        // populating the points
        for (const auto &point : data.frame_points)
            frames.emplace_back(point);

        // remove oldest points
        if (frame_sizes.size() > online_sub_frame_trail)
        {
            size_t size_to_rem = frame_sizes.front();
            frame_sizes.pop_front();

            for (size_t idx = 0; idx < size_to_rem; ++idx)
                frames.pop_front();
        }

        // publishes a window of frames
        curr_frame_publisher.publish(*VisualFuncs<T>::eig_to_pc_2(frames, frames.size(), c_header));
    }

    if (!data.pts.empty()) // map_publisher
        map_publisher.publish(*VisualFuncs<T>::eig_to_pc_2(data.pts, c_header));
}

template <typename T>
SE3Type<T> Visualizer<T>::lookup_transform(const std::string &target_frame, const std::string &source_frame)
{
    std::string err_msg;

    if (tf2_buffer._frameExists(source_frame) &&
        tf2_buffer._frameExists(target_frame) &&
        tf2_buffer.canTransform(target_frame, source_frame, ros::Time(0), &err_msg))
    {
        try
        {
            auto tf = tf2_buffer.lookupTransform(target_frame, source_frame, ros::Time(0));
            return VisualFuncs<T>::transform_to_sophus(tf);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }

    ROS_WARN(
        "Failed to find tf between %s and %s. Reason=%s", target_frame.c_str(),
        source_frame.c_str(), err_msg.c_str());

    return {};
}

template struct Visualizer<double>;
template struct Visualizer<float>;
