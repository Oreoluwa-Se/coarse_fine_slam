#include "coarse_fine_slam/helpers/preprocess/lidar.hpp"
#include "coarse_fine_slam/helpers/calc_helpers.hpp"
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <cmath>
#include <algorithm>
#include <tbb/concurrent_hash_map.h>
#include <set>

namespace IncrementalDownsample
{
    template <typename T>
    void VoxelData<T>::add_info(PointInfo<T> &&val)
    {
        stats.add_info(val.pts);
        points.emplace(val.squared_dist, std::move(val));
    }

    template <typename T>
    void VoxelData<T>::sampled_points(std::unordered_map<T, Point3dPtrVectCC<T>> &sampled, T count)
    {
        // Determine the number of points to sample
        size_t sample_count = std::max<size_t>(1, static_cast<size_t>(count));
        size_t step = points.size() / sample_count;
        if (step == 0)
            return;

        // Iterate over points and add sampled points
        auto it = points.begin();
        for (size_t idx = 0; idx < sample_count && it != points.end(); ++idx, std::advance(it, step))
        {
            PointInfo<T> &pi = it->second;
            auto new_p = std::make_shared<Point3d<T>>(pi.pts, pi.intensity, pi.timestamp);
            new_p->vox = pi.vox;

            auto &map_key = sampled[new_p->timestamp];
            map_key.emplace_back(std::move(new_p));
        }
    }
    template struct VoxelData<double>;
    template struct VoxelData<float>;

    template <typename T>
    void DownsampleBlock<T>::add_point(PointInfo<T> &&val)
    {
        // add to the pointer list
        val.vox = Point3d<T>::calc_vox_index(val.pts, params<T>.build.voxel_size);
        auto &voxel_data = points[val.vox];
        voxel_data.add_info(std::move(val));
    }

    template <typename T>
    std::pair<T, T> DownsampleBlock<T>::total_info()
    {
        int map_size = points.size();
        variances = Eigen::Matrix<T, 1, Eigen::Dynamic>::Zero(1, map_size);
        counts = Eigen::Matrix<T, 1, Eigen::Dynamic>::Zero(1, map_size);
        int idx = 0;
        for (auto &pair : points)
        {
            variances(idx) = pair.second.stats.max_variance_val();
            counts(idx) = pair.second.stats.count;
            ++idx;
        }

        return {variances.sum(), counts.sum()};
    }

    template <typename T>
    std::unordered_map<T, Point3dPtrVectCC<T>> DownsampleBlock<T>::reduced(T count)
    {
        // scale by maximum [we use information about spread and density]
        Eigen::Matrix<T, 1, Eigen::Dynamic> weight = vox_weight_generator(variances, counts, count);

        std::unordered_map<T, Point3dPtrVectCC<T>> ret_points;
        int idx = 0;
        for (auto &pair : points)
        {
            pair.second.sampled_points(ret_points, weight(idx));
            ++idx;
        }

        return ret_points;
    }

    template struct DownsampleBlock<double>;
    template struct DownsampleBlock<float>;
};

namespace Lidar
{
    template <typename T>
    void ProcessedFrame<T>::agg_voxel_stats(
        Eigen::Matrix<T, 1, Eigen::Dynamic> &var, Eigen::Matrix<T, 1, Eigen::Dynamic> &counts,
        std::array<IncrementalDownsample::DownsampleBlock<T>, 8> &to_flushs)
    {
        var = Eigen::Matrix<T, 1, 8>::Zero();
        counts = Eigen::Matrix<T, 1, 8>::Zero();
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, to_flushs.size()),
            [&](tbb::blocked_range<size_t> &range)
            {
                for (size_t idx = range.begin(); idx != range.end(); ++idx)
                {
                    T variance, count;
                    std::tie(variance, count) = to_flushs[idx].total_info();
                    var(idx) = variance;
                    counts(idx) = count;
                }
            });
    }

    template <typename T>
    Eigen::Matrix<T, 1, Eigen::Dynamic> ProcessedFrame<T>::generate_split(
        size_t vector_size, T dwnsample_ratio,
        std::array<IncrementalDownsample::DownsampleBlock<T>, 8> &o_points)
    {
        // get spread and count for all voxels
        Eigen::Matrix<T, 1, Eigen::Dynamic> variances, counts;
        ProcessedFrame<T>::agg_voxel_stats(variances, counts, o_points);

        // scale by maximum[we use information about spread and density]
        T d_size = dwnsample_ratio * static_cast<T>(vector_size);
        return vox_weight_generator(variances, counts, d_size);
    }

    template <typename T>
    void ProcessedFrame<T>::populate_seq()
    {
        Eigen::Matrix<T, 1, Eigen::Dynamic> weight = ProcessedFrame<T>::generate_split(valid_points_count, params<T>.lidar.downsample_ratio, to_flushs);

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, to_flushs.size()),
            [&](const tbb::blocked_range<size_t> &range)
            {
                for (size_t i = range.begin(); i != range.end(); ++i)
                {
                    // std::unordered_map<T, Point3dPtrVectCC<T>>
                    auto res = to_flushs[i].reduced(std::max(T(1.0), weight(i)));

                    for (auto &pi : res)
                    {
                        typename ConcurrentMap::accessor accessor;
                        if (stack.insert(accessor, pi.first))
                            accessor->second = std::make_shared<Frame::Lidar<T>>(pi.first);

                        // insert point
                        std::move(
                            pi.second.begin(), pi.second.end(),
                            std::back_inserter(accessor->second->frame));
                    }
                }
            });

        // marker for end of imu dat
        check_timestamp = *std::prev(ts.end());
    }

    template <typename T>
    Frame::BasePtr<T> ProcessedFrame<T>::get_next()
    {
        if (!iter_began)
        {
            curr_lidar_iter = ts.begin();
            iter_began = true;
        }

        typename ConcurrentMap::accessor accessor;
        if (curr_lidar_iter != ts.end() && stack.find(accessor, *curr_lidar_iter))
        {
            ++curr_lidar_iter;
            return accessor->second;
        }

        return nullptr;
    }

    template <typename T>
    bool ProcessedFrame<T>::empty() { return curr_lidar_iter == ts.end(); }

    template struct ProcessedFrame<double>;
    template struct ProcessedFrame<float>;

    template <typename T>
    SphericalCoords<T> Preprocess<T>::sphere_conversion(const Eigen::Matrix<T, 3, 1> &point, T dist_sq)
    {
        SphericalCoords<T> spherical;
        T d_sq = dist_sq == -1.0 ? point.squaredNorm() : dist_sq;
        spherical.r = std::sqrt(d_sq);
        spherical.theta = std::atan2(point[1], point[0]);  // azimuth angle
        spherical.phi = std::asin(point[2] / spherical.r); // elevation angle
        return spherical;
    }

    template <typename T>
    T Preprocess<T>::calc_point_time(std::vector<bool> &is_first, std::vector<T> &yaw_fp, std::vector<T> &yaw_last, std::vector<T> &time_last, T &curr_time, int layer, T yaw_angle)
    {
        T omega = 0.361 * params<T>.lidar.freq;
        T point_time = 0.0;
        if (is_first[layer])
        {
            yaw_fp[layer] = yaw_angle;
            is_first[layer] = false;
            yaw_last[layer] = yaw_angle;

            return 0.0; // Assuming the first point has time 0.0
        }

        point_time = yaw_angle <= yaw_fp[layer] ? yaw_fp[layer] - yaw_angle : yaw_fp[layer] - yaw_angle + 2 * M_PI;
        point_time /= omega;

        if (point_time < time_last[layer])
            point_time += 2 * M_PI / omega;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = point_time;

        // This is the time from first scan to current point
        point_time += curr_time;
        return point_time;
    }

    template <typename T>
    ProcessedFramePtr<T> Preprocess<T>::process_frame(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        T curr_time = msg->header.stamp.toSec();

        // Continue from here next
        pcl::PointCloud<robosense_ros::Point>::Ptr cloud(new pcl::PointCloud<robosense_ros::Point>);
        pcl::fromROSMsg(*msg, *cloud);

        std::vector<bool> is_first(params<T>.lidar.num_scan_lines, true);
        std::vector<T> yaw_fp(params<T>.lidar.num_scan_lines, 0.0);
        std::vector<T> yaw_last(params<T>.lidar.num_scan_lines, 0.0);
        std::vector<T> time_last(params<T>.lidar.num_scan_lines, 0.0);

        const int cloud_size = cloud->points.size();
        bool has_offset_time = !cloud->points.empty() && cloud->points.back().timestamp > 0;

        T min_sq_range = params<T>.lidar.min_range * params<T>.lidar.min_range;
        T max_sq_range = params<T>.lidar.max_range * params<T>.lidar.max_range;

        ProcessedFramePtr<T> store = std::make_shared<ProcessedFrame<T>>();

        for (int idx = 0; idx < cloud_size; ++idx)
        {
            const auto &point = cloud->points[idx];
            if (!pcl::isFinite(point))
                continue;

            T dist_sq = point.x * point.x + point.y * point.y + point.z * point.z;
            if (dist_sq < min_sq_range || dist_sq > max_sq_range)
                continue;

            T point_time = 0.0;
            if (!has_offset_time)
            {
                T yaw_angle = atan2(point.y, point.x);
                point_time = Preprocess<T>::calc_point_time(is_first, yaw_fp, yaw_last, time_last, curr_time, point.ring, yaw_angle);
            }
            else
                point_time = point.timestamp; // working in seconds

            IncrementalDownsample::PointInfo<T> pi;
            {
                pi.pts = Eigen::Matrix<T, 3, 1>(point.x, point.y, point.z);
                pi.squared_dist = dist_sq;
                pi.timestamp = point_time;
                pi.octant_key = Point3d<T>::sign_cardinality(pi.pts);
                pi.intensity = point.intensity;
            }

            // track unique timestamps
            store->ts.insert(pi.timestamp);

            // segment stuff for cardinality
            store->to_flushs[pi.octant_key].add_point(std::move(pi));
            ++store->valid_points_count;
        }

        return store;
    }

    template struct Preprocess<double>;
    template struct Preprocess<float>;
};
