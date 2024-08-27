#include "coarse_fine_slam/helpers/visual_helper.hpp"
#include <regex>
namespace
{
    inline std::string FixFrameId(const std::string &frame_id)
    {
        return std::regex_replace(frame_id, std::regex("^/"), "");
    }

}

template <typename T>
geometry_msgs::Transform VisualFuncs<T>::sophus_to_transform(const SE3Type<T> &pose)
{
    geometry_msgs::Transform t;
    t.translation.x = pose.translation().x();
    t.translation.y = pose.translation().y();
    t.translation.z = pose.translation().z();

    Eigen::Quaternion<T> q(pose.so3().unit_quaternion());
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    t.rotation.w = q.w();

    return t;
}

template <typename T>
geometry_msgs::Pose VisualFuncs<T>::sophus_to_pose(const SE3Type<T> &pose)
{
    geometry_msgs::Pose t;
    t.position.x = pose.translation().x();
    t.position.y = pose.translation().y();
    t.position.z = pose.translation().z();

    Eigen::Quaternion<T> q(pose.so3().unit_quaternion());
    t.orientation.x = q.x();
    t.orientation.y = q.y();
    t.orientation.z = q.z();
    t.orientation.w = q.w();

    return t;
}

template <typename T>
SE3Type<T> VisualFuncs<T>::transform_to_sophus(const geometry_msgs::TransformStamped &transform)
{
    const auto &t = transform.transform;

    Eigen::Quaternion<T> quat(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    Eigen::Matrix<T, 3, 1> translation(t.translation.x, t.translation.y, t.translation.z);

    // Construct the Sophus SE3 type using the quaternion and translation vector
    return SE3Type<T>(quat, translation);
}

template <typename T>
std::unique_ptr<sensor_msgs::PointCloud2> VisualFuncs<T>::create_pc2_msg(const size_t n_points, const std_msgs::Header &header, bool timestamp)
{
    auto msg = std::make_unique<sensor_msgs::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*msg);

    msg->header = header;
    msg->header.frame_id = FixFrameId(msg->header.frame_id);
    msg->fields.clear();

    int offset = 0;
    offset = addPointField(*msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset += sizeOfPointField(sensor_msgs::PointField::FLOAT32);
    if (timestamp)
    {
        // assuming timestamp on a velodyne fashion for now (between 0.0 and 1.0) --> NEED TO FIX LATER IF USING
        offset = addPointField(*msg, "time", 1, sensor_msgs::PointField::FLOAT64, offset);
        offset += sizeOfPointField(sensor_msgs::PointField::FLOAT64);
    }

    msg->point_step = offset;
    msg->row_step = msg->width * msg->point_step;
    msg->data.resize(msg->height * msg->row_step);
    modifier.resize(n_points);

    return msg;
}

template <typename T>
void VisualFuncs<T>::fill_point_cloud(const Point3dWPtrVecCC<T> &points, sensor_msgs::PointCloud2 &msg)
{
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");

    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z)
    {
        if (auto sp = points[i].lock())
        {
            *msg_x = static_cast<float>(sp->x());
            *msg_y = static_cast<float>(sp->y());
            *msg_z = static_cast<float>(sp->z());
        }
    }
}

template <typename T>
void VisualFuncs<T>::fill_point_cloud(const std::deque<Eigen::Matrix<T, 3, 1>> &points, sensor_msgs::PointCloud2 &msg)
{
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");

    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z)
    {
        *msg_x = static_cast<float>(points[i](0));
        *msg_y = static_cast<float>(points[i](1));
        *msg_z = static_cast<float>(points[i](2));
    }
}

template <typename T>
void VisualFuncs<T>::fill_point_cloud(const std::deque<Point3dWPtr<T>> &points, sensor_msgs::PointCloud2 &msg)
{
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");

    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z)
    {
        if (auto sp = points[i].lock())
        {
            *msg_x = static_cast<float>(sp->x());
            *msg_y = static_cast<float>(sp->y());
            *msg_z = static_cast<float>(sp->z());
        }
    }
}

template <typename T>
std::unique_ptr<sensor_msgs::PointCloud2> VisualFuncs<T>::eig_to_pc_2(const std::deque<Point3dWPtr<T>> &points, size_t size, const std_msgs::Header &header)
{
    auto msg = create_pc2_msg(size, header);
    fill_point_cloud(points, *msg);

    return msg;
}

template <typename T>
std::unique_ptr<sensor_msgs::PointCloud2> VisualFuncs<T>::eig_to_pc_2(const std::deque<Eigen::Matrix<T, 3, 1>> &points, size_t size, const std_msgs::Header &header)
{
    auto msg = create_pc2_msg(points.size(), header);
    fill_point_cloud(points, *msg);

    return msg;
}

template <typename T>
std::unique_ptr<sensor_msgs::PointCloud2> VisualFuncs<T>::eig_to_pc_2(const Point3dWPtrVecCC<T> &points, const std_msgs::Header &header)
{
    auto msg = create_pc2_msg(points.size(), header);
    fill_point_cloud(points, *msg);

    return msg;
}

template struct VisualFuncs<double>;
template struct VisualFuncs<float>;
