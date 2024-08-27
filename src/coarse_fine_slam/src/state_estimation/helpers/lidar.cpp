#include "coarse_fine_slam/state_estimation/helpers/lidar.hpp"
#include <Eigen/Dense>
#include <cstdlib>
#include <iostream>
#include <iostream>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/parallel_invoke.h>
#include <tbb/spin_mutex.h>
#include <unordered_map>
#include <typeinfo>
#include <tbb/concurrent_hash_map.h>
#include <atomic>

namespace
{
    // window size used for calculating new search radius
    constexpr size_t RADIUS_WINDOW = 20;
}

template <typename T>
LidarKf<T>::LidarKf(const Kalman::StatePtr<T> &state, const PointStoragePtr<T> &map, const Kalman::FilterPtr<T> &kf)
    : state(state), map(map), kf(kf), radius_track(std::make_unique<ScalarAveraging<T>>(RADIUS_WINDOW))
{
    // lidar noise
    lidar_noise = kf->noise_scale * std::pow(params<T>.lidar.measurement_noise, 2);

    // parameters that look to help make seach adaptable
    search_radius = params<T>.lidar.search_range;
    radius_track->updated_matrix(search_radius); // windowed ema approach
    importance_criteria = 0.7;
}

template <typename T>
void LidarKf<T>::undistort_and_stack_points(Point3dPtrVectCC<T> &frame_points, Frame::LidarPtr<T> &frame, T base_timestamp)
{
    SE3Type<T> wp_base, wc_base_inv;
    {
        const auto pred_state = state->predict_next(frame->timestamp - base_timestamp);
        const SE3Type<T> pred_world_imu = SE3Type<T>(pred_state.rot_pred, pred_state.p_pred);

        const SE3Type<T> imu_lidar = state->get_pose(StateRef::PoseType::Imu_Lidar);
        const SE3Type<T> curr_world_imu = state->get_pose(StateRef::PoseType::World_Imu);

        // wp_base: current base -> predicted frame
        wp_base = pred_world_imu * imu_lidar;

        // wc_base: current base -> current frame
        const SE3Type<T> wc_base = curr_world_imu * imu_lidar;
        wc_base_inv = wc_base.inverse();
    }

    // double downsampling scheme like Kiss-Icp paper: https://arxiv.org/pdf/2209.15397
    T dd_voxel = 1.0 + params<T>.build.voxel_size;

    // track total points
    total_frame_points += frame->frame.size();
    for (auto &point : frame->frame)
    {
        // transform sensor point -> predicted world frame
        Eigen::Matrix<T, 3, 1> adjusted_point = wp_base * point->point;
        // This goes from predicted world frame -> current frame
        point->point = wc_base_inv * adjusted_point;

        // adjust other parameters
        point->octant_key = Point3d<T>::sign_cardinality(point->point);
        point->vox = Point3d<T>::calc_vox_index(point->point, params<T>.build.voxel_size);

        // apply double sampling
        auto &map_info = points_cluster[Point3d<T>::calc_vox_index(point->point, dd_voxel)];
        if (map_info.second == 0)
        {
            first_points.emplace_back(point->point);
            map_info.first.setZero();
        }

        // tallying to calculate average
        map_info.first += point->point;
        ++map_info.second;

        // tallying points
        frame_points.emplace_back(std::move(point));
    }
}

template <typename T>
void LidarKf<T>::averaged_points()
{
    /*
     * For a good representative pattern: first point + mean of points (coarse-fine mix) is how we represent the pointcloud
     * We perform icp by starting with the coarse pattern for initial icp.. then switching to the fine [firsts] for fine-tuning
     * We can get good pattern representation for the pointcloud also avoid the local-minima problem. These become the base points
     * First points already accounted for in the undistort_and_stack function.. now we average
     */
    size_t map_size = points_cluster.size();
    avg_points.clear();
    avg_points.resize(map_size);

    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, map_size),
        [&](const tbb::blocked_range<std::size_t> &r)
        {
            // Get the iterator at position i
            for (size_t idx = r.begin(); idx < r.end(); ++idx)
            {
                auto it = points_cluster.begin();
                std::advance(it, idx); // O(n) but n is "small" maybe not a bother?

                // Store the averaged points in the second half of the arrays
                avg_points[idx] = it->second.first / it->second.second;
            }
        });

    points_cluster.clear(); // memory dump
}

template <typename T>
AVector3TVec<T> LidarKf<T>::get_viz_points() { return key_points; }

template <typename T>
void LidarKf<T>::clear_cycle_info()
{
    points_cluster.clear();
    first_points.clear();
    avg_points.clear();

    total_frame_points = 0;
}

template <typename T>
bool LidarKf<T>::run()
{
    averaged_points();    // coarse points for initial alignments
    state->clone_state(); // clones the current predicted state
    kf->prev_cost.setZero();
    IteratorResult<T> res;
    size_t prev_size = 0, idx = 0, row_dim = 0;

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> noise, noise_inv;
    T total_jaccard = 0.0, proximity_ratio = 0.0;

    for (; idx < params<T>.num_iter; ++idx)
    {
        Tracker<T> sd;
        res = iterator_steps(idx);
        total_jaccard += res.jaccard_res;
        proximity_ratio += res.proximity_ratio;

        if (MatchingResult::MapPoints == res.match_type)
        {
            sd.converge = false;
            break;
        }

        h_model_build(res.matched, sd);
        row_dim = sd.jacob.rows();

        if (prev_size != row_dim)
        {
            noise = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Identity(row_dim, row_dim);
            noise.diagonal() *= lidar_noise;
            noise_inv = noise.inverse();
            prev_size = row_dim;
        }

        if (kf->step(sd, noise, noise_inv, (idx + 1) == params<T>.num_iter, false))
            break;
    }

    // calculate average iou over duration
    T avg_jacard = total_jaccard / T(idx + 1);
    T avg_prox = proximity_ratio / T(idx + 1);
    update_search_range(avg_jacard);

    // prepping for visualization - only insert keyframes
    if (params<T>.visualize_subframes)
        extract_key_points(res.matched);

    res.matched.clear();

    return params<T>.tight_budget ? avg_prox <= importance_criteria : true;
}

template <typename T>
void LidarKf<T>::update_search_range(T avg_iou)
{
    /*
     * 0.5 considered good. If we have highly overlapping clouds.. we can reduce the search radius for next cycle
     * When we have weak overlap... we increase the search radius for search.
     */

    T adjust_scale = (1.0 + (0.5 - avg_iou));
    T curr_frame_adjustment = search_radius * adjust_scale;
    search_radius = std::max(T(0.5), radius_track->updated_matrix(curr_frame_adjustment));
    search_radius = std::min(search_radius, params<T>.lidar.search_range);
}

template <typename T>
void LidarKf<T>::extract_key_points(tbb::concurrent_vector<H_modeller<T>> &matched)
{
    key_points.resize(matched.size());

    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, matched.size()),
        [&](const tbb::blocked_range<std::size_t> &r)
        {
            for (size_t idx = r.begin(); idx < r.end(); ++idx)
                key_points[idx] = std::move(matched[idx].world_coord);
        });
}

template <typename T>
void LidarKf<T>::quartile_refinement(tbb::concurrent_vector<T> &weights, tbb::concurrent_vector<H_modeller<T>> &matched, IteratorResult<T> &res)
{
    // we want to avoid points that the "spread" or in this case average distance between nearest points is high
    size_t points_size = weights.size();

    T upper_q;
    {
        T lower_q;
        std::tie(lower_q, upper_q) = RandomFuncs<T>::outlier_check(weights);

        weights.clear();
    }

    res.matched.reserve(points_size);
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, points_size),
        [&](const tbb::blocked_range<std::size_t> &r)
        {
            for (size_t idx = r.begin(); idx < r.end(); ++idx)
            {
                if (matched[idx].weight <= upper_q)
                    res.matched.emplace_back(std::move(matched[idx]));
            }
        });
}

template <typename T>
IteratorResult<T> LidarKf<T>::iterator_steps(size_t idx)
{
    // get world coordinates
    const auto &points = idx == 0 ? avg_points : first_points;

    SE3Type<T> world_lidar = state->lidar_imu_world();
    SE3Type<T> imu_lidar = state->get_pose(StateRef::PoseType::Imu_Lidar);
    std::atomic<size_t> close_count{0};

    tbb::concurrent_vector<H_modeller<T>> matched;
    tbb::concurrent_vector<T> disparity_weights;

    matched.reserve(points.size());
    disparity_weights.reserve(points.size());

    // Gather matched points
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, points.size()),
        [&](const tbb::blocked_range<std::size_t> &r)
        {
            for (size_t idx = r.begin(); idx < r.end(); ++idx)
            {
                Eigen::Matrix<T, 3, 1> world_pt = world_lidar * points[idx];
                SearchRunner<T> sr = map->knn_search(world_pt, params<T>.lidar.num_nearest, search_radius, SearchType::Point);
                if (sr.enough_points())
                {
                    auto matched_point = H_modeller<T>(sr.qp, sr.points);
                    if (matched_point.valid)
                    {
                        matched_point.imu_coord = imu_lidar * points[idx];
                        if (params<T>.online_calib)
                            matched_point.src_coord = points[idx];

                        // moving stuff appropriately
                        matched.emplace_back(std::move(matched_point));
                        disparity_weights.emplace_back(matched_point.weight);

                        if (matched_point.close_proximity)
                            ++close_count;
                    }
                }
            }
        });

    IteratorResult<T> res;
    res.jaccard_res = static_cast<T>(matched.size()) / static_cast<T>(points.size());
    res.proximity_ratio = static_cast<T>(close_count.load()) / static_cast<T>(points.size());

    // Gather matched and valid points
    quartile_refinement(disparity_weights, matched, res);

    // verification metric - need more than one match
    if (idx == 0)
    {
        avg_points.clear();
        if (res.matched.size() <= 1)
            res.match_type = MatchingResult::MapPoints;
    }

    return res;
}

template <typename T>
void LidarKf<T>::h_model_build(tbb::concurrent_vector<H_modeller<T>> &matched, Tracker<T> &sd)
{
    sd.clear(); // reset variables

    const int m_size = matched.size();
    const size_t num_rows = m_size;

    // memory allocations kalman filter matrices
    sd.jacob = Eigen::Matrix<T, Eigen::Dynamic, StateRef::TOTAL_DIM>::Zero(num_rows, StateRef::TOTAL_DIM);
    sd.z = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(num_rows, 1);

    // Rotation from lidar to imu
    const Eigen::Matrix<T, 3, 3> lidar_b_rot_inv = state->curr_lidar_base_rot.transpose();
    const Eigen::Matrix<T, 3, 3> world_lidar = state->lidar_imu_world().rotationMatrix();
    const Eigen::Matrix<T, 3, 1> lidar_sensor_pos = state->get_state_info("POS_BL");

    auto sigmoid = [](auto value)
    {
        return static_cast<T>(1) / (static_cast<T>(1) + std::exp(-value));
    };

    // support for each row
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, m_size),
        [&](tbb::blocked_range<size_t> &r)
        {
            for (size_t idx = r.begin(); idx < r.end(); ++idx)
            {
                auto &c_matched = matched[idx];

                // uncertain points have less an effect using sigmoid as a bound manager
                T weight = sigmoid(1.0 / c_matched.weight);

                Eigen::Matrix<T, 1, 3> surf_norm_t = c_matched.s_norm.transpose();

                // Case 1: dy/du => Jacobian of equation with respect to quaternion
                StateRef::VarInfo spi = get_var_info("ORI");
                sd.jacob.row(idx).block(0, spi.index, 1, spi.size) = -surf_norm_t * state->curr_rot * PoseFuncs<T>::skew_matrix(c_matched.imu_coord);

                // Case 2: dy/dp => Derivative wrt to external pose
                spi = get_var_info("POS");
                sd.jacob.row(idx).block(0, spi.index, 1, spi.size) = surf_norm_t;

                if (params<T>.online_calib)
                {
                    const Eigen::Matrix<T, 3, 1> val = lidar_b_rot_inv * (c_matched.src_coord - lidar_sensor_pos);
                    const Eigen::Matrix<T, 3, 3> val_m = state->curr_rot * state->curr_imu_base_rot * PoseFuncs<T>::skew_matrix(val);

                    // Case 4a: dy/d(ori_bl) => Jacobian of equation with respect to lidar sensor orientation
                    {
                        spi = get_var_info("ORI_BL");
                        sd.jacob.row(idx).block(0, spi.index, 1, spi.size) = surf_norm_t * val_m;

                        // Case 4b: dy/dt_pos_bl => Jacobian of equation with respect to lidar sensor translation
                        spi = get_var_info("POS_BL");
                        sd.jacob.row(idx).block(0, spi.index, 1, spi.size) = -surf_norm_t * world_lidar;
                    }

                    // Case 5a: dy/d(ori_bl) => Jacobian of equation with respect to imu sensor orientation
                    if (params<T>.imu_en)
                    {
                        spi = get_var_info("ORI_BI");
                        sd.jacob.row(idx).block(0, spi.index, 1, spi.size) = -surf_norm_t * val_m;

                        // Case 4b: dy/dt_pos_bl => Jacobian of equation with respect to lidar sensor translation
                        spi = get_var_info("POS_BI");
                        sd.jacob.row(idx).block(0, spi.index, 1, spi.size) = surf_norm_t * state->curr_rot;
                    }
                }

                // residual error
                sd.jacob.row(idx) *= weight;
                sd.z(idx) = -c_matched.dist_to_plane * weight;
            }
        });
}

template struct LidarKf<double>;
template struct LidarKf<float>;