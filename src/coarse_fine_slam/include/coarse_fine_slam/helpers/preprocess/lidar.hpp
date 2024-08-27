#ifndef LIDAR_PREPROCESS_HPP
#define LIDAR_PREPROCESS_HPP

#include "lidar_structs.hpp"
#include "merged_data.hpp"
#include "parameters.hpp"
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <map_storage/sections/kd_tree/specialized.hpp>
#include <map>
#include <unordered_map>
#include <tbb/concurrent_hash_map.h>
#include <set>

namespace IncrementalDownsample
{
    template <typename T>
    struct PointInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix<T, 3, 1> pts;
        Eigen::Vector3i vox;
        T timestamp;
        int octant_key;
        T squared_dist;
        std::uint8_t intensity;
    };

    template <typename T>
    struct VoxelData
    {
        VoxelData() = default;

        void add_info(PointInfo<T> &&val);

        void sampled_points(std::unordered_map<T, Point3dPtrVectCC<T>> &sampled, T count);

        std::multimap<T, PointInfo<T>> points;

        RunningStats<T> stats;
    };

    template <typename T>
    struct DownsampleBlock
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using VoxelHMap = std::unordered_map<Eigen::Vector3i, VoxelData<T>, VoxelHash, VoxelHashEqual>;

        void add_point(PointInfo<T> &&point);

        std::unordered_map<T, Point3dPtrVectCC<T>> reduced(T count);

        std::pair<T, T> total_info();

        VoxelHMap points;
        Eigen::Matrix<T, 1, Eigen::Dynamic> variances, counts;
    };
};

namespace Lidar
{

    template <typename T>
    struct ProcessedFrame
    {
        using Ptr = std::shared_ptr<ProcessedFrame<T>>;

        using ConcurrentMap = tbb::concurrent_hash_map<T, Frame::LidarPtr<T>>;
        ConcurrentMap stack;
        std::set<T> ts;
        size_t valid_points_count = 0;
        std::array<IncrementalDownsample::DownsampleBlock<T>, 8> to_flushs;
        typename std::set<T>::iterator curr_lidar_iter;

        T check_timestamp = 0.0;
        bool iter_began = false;

        static void agg_voxel_stats(
            Eigen::Matrix<T, 1, Eigen::Dynamic> &var, Eigen::Matrix<T, 1, Eigen::Dynamic> &counts,
            std::array<IncrementalDownsample::DownsampleBlock<T>, 8> &to_flush);

        static Eigen::Matrix<T, 1, Eigen::Dynamic> generate_split(
            size_t size, T dwnsample_ratio,
            std::array<IncrementalDownsample::DownsampleBlock<T>, 8> &o_points);

        void populate_seq();

        Frame::BasePtr<T> get_next();

        bool empty();
    };

    template <typename T>
    using ProcessedFramePtr = typename ProcessedFrame<T>::Ptr;

    template <typename T>
    struct SphericalCoords
    {
        T r, theta, phi;
    };

    template <typename T>
    struct Preprocess
    {
        static SphericalCoords<T> sphere_conversion(const Eigen::Matrix<T, 3, 1> &point, T dist_sq = -1.0);

        static ProcessedFramePtr<T> process_frame(const sensor_msgs::PointCloud2::ConstPtr &msg);

        static T calc_point_time(
            std::vector<bool> &is_first, std::vector<T> &yaw_fp, std::vector<T> &yaw_last,
            std::vector<T> &time_last, T &curr_time, int layer, T yaw_angle);
    };

};

#endif