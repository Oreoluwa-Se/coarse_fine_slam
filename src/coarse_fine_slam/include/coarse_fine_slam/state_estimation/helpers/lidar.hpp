#ifndef LIDAR_ESTIMATION_HPP
#define LIDAR_ESTIMATION_HPP

#include "coarse_fine_slam/helpers/calc_helpers.hpp"
#include "coarse_fine_slam/state_estimation/helpers/kf.hpp"
#include "coarse_fine_slam/helpers/preprocess/merged_data.hpp"
#include "parameters.hpp"
#include "h_modeller.hpp"
#include "state.hpp"

enum class MatchingResult
{
    MapPoints,
    KalmanfilterSteps,
    None
};

template <typename T>
struct IteratorResult
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    tbb::concurrent_vector<H_modeller<T>> matched;
    MatchingResult match_type = MatchingResult::KalmanfilterSteps;
    T jaccard_res = 0;
    T proximity_ratio = 0;
};

template <typename T>
struct LidarKf
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<LidarKf<T>>;

    using PointClusterType = std::unordered_map<Eigen::Vector3i, std::pair<Eigen::Matrix<T, 3, 1>, T>, VoxelHash>;

    LidarKf(const Kalman::StatePtr<T> &state, const PointStoragePtr<T> &map, const Kalman::FilterPtr<T> &kf);

    bool run();

    void undistort_and_stack_points(Point3dPtrVectCC<T> &frame_points, Frame::LidarPtr<T> &frame, T base_timestamp);

    void clear_cycle_info();

    AVector3TVec<T> get_viz_points();

private: // function
    void update_search_range(T avg_iou);

    IteratorResult<T> iterator_steps(size_t idx);

    void h_model_build(tbb::concurrent_vector<H_modeller<T>> &matched, Tracker<T> &sd);

    void averaged_points();

    void extract_key_points(tbb::concurrent_vector<H_modeller<T>> &matched);

    void quartile_refinement(tbb::concurrent_vector<T> &weights, tbb::concurrent_vector<H_modeller<T>> &matched, IteratorResult<T> &refined);

private: // attributes
    Kalman::StatePtr<T> state = nullptr;
    PointStoragePtr<T> map = nullptr;
    Kalman::FilterPtr<T> kf = nullptr;
    SAPtr<T> radius_track = nullptr;

    // Search parameters and tuning
    T lidar_noise, search_radius, importance_criteria;

    // helper funcs - lidar coordinate can be first_points or avg_points
    AVector3TVec<T> first_points, avg_points, world_coord, key_points;
    PointClusterType points_cluster;
    size_t total_frame_points = 0;
};

template <typename T>
using LidarKfPtr = typename LidarKf<T>::Ptr;

#endif