#include "coarse_fine_slam/state_estimation/interface.hpp"
#include <iomanip>
#include <algorithm>
#include <iostream>
#include <unordered_map>

template <typename T>
EstimationInterface<T>::EstimationInterface()
    : map(std::make_shared<PointStorage<T>>(
          params<T>.build.max_points_in_vox,
          params<T>.build.max_points_in_oct_layer,
          params<T>.build.imbal_factor,
          params<T>.build.del_nodes_factor,
          params<T>.build.track_stats,
          params<T>.build.init_map_size,
          params<T>.build.voxel_size)),
      state(std::make_shared<Kalman::State<T>>()),
      kf(std::make_shared<Kalman::Filter<T>>(state)),
      lkf(std::make_shared<LidarKf<T>>(state, map, kf))
{
    del_typ = params<T>.box_trim ? DeleteType::Box : DeleteType::Spherical;
    if (params<T>.imu_en)
        ikf = std::make_shared<ImuKf<T>>(state, kf);

    cycle_tracker = EndofCycleResults<T>();
}

template <typename T>
bool EstimationInterface<T>::build_init()
{
    Point3dPtrVect<T> points;
    {
        points.reserve(frame_points.size());
        for (auto &pts : frame_points)
            points.emplace_back(std::move(pts));
    }

    map->build(points);
    frame_points.clear();
    lkf->clear_cycle_info();

    return true;
}

template <typename T>
void EstimationInterface<T>::init_orientation()
{
    if (params<T>.imu_en)
    {
        state->set_state("PRED_ACC", params<T>.imu.mean_acc);
        state->set_state("GRAV", -params<T>.imu.mean_acc);
    }
    else
    {
        Eigen::Matrix<T, 3, 1> g;
        (g << 0.0, 0.0, -params<T>.gravity).finished();
        state->set_state("GRAV", g);

        state->set_state("PRED_ACC", -g);
    }

    state->set_init_orientation();
    orientation_init = true;
}

template <typename T>
void EstimationInterface<T>::update_base_timestamp(T bt)
{
    // get's updated either at the begining of each stage or when we have readings from a different sensor
    base_timestamp = bt;
}

template <typename T>
T EstimationInterface<T>::get_base_timestamp()
{
    return base_timestamp;
}

template <typename T>
void EstimationInterface<T>::lidar_frame_stack(Frame::LidarPtr<T> &frame)
{
    if (frame->frame.empty())
        return;

    // update calculated frames and stacking the points to transform later
    lkf->undistort_and_stack_points(frame_points, frame, base_timestamp);
}

template <typename T>
bool EstimationInterface<T>::lidar_run(bool visualize_full_map)
{
    if (frame_points.empty())
        return false;

    if (lkf->run()) // Returns a boolean that determines if we insert the map or not
    {
        Point3dPtrVect<T> points_to_insert;
        TransformFuncs<T>::moved_transform(state->lidar_imu_world(), frame_points, points_to_insert, params<T>.build.voxel_size);
        map->insert(points_to_insert);
    }

    // can also pull the total points if visualizing data
    if (visualize_full_map || params<T>.visualize_subframes)
    {
        cycle_tracker.reset();

        if (!visualize_full_map && params<T>.visualize_subframes)
            cycle_tracker.frame_points = lkf->get_viz_points(); // key points used for icp calculations

        // visuzling entire_map
        if (visualize_full_map)
            cycle_tracker.pts = map->get_points();

        // return the current pose - and orientation
        cycle_tracker.curr_pose = state->get_pose(StateRef::PoseType::World_Imu);
        cycle_tracker.pose_cov.template block<3, 3>(0, 0) = kf->get_P_block("POS", "POS");
        cycle_tracker.pose_cov.template block<3, 3>(0, 3) = kf->get_P_block("POS", "ORI");
        cycle_tracker.pose_cov.template block<3, 3>(3, 0) = kf->get_P_block("ORI", "POS");
        cycle_tracker.pose_cov.template block<3, 3>(3, 3) = kf->get_P_block("ORI", "ORI");
    }

    // clear for next sprint
    frame_points.clear();
    lkf->clear_cycle_info();

    return true;
}

template <typename T>
void EstimationInterface<T>::imu_run(const Eigen::Matrix<T, 3, 1> &acc, const Eigen::Matrix<T, 3, 1> &gyro)
{
    // run the imu kalman filter stuff
    ikf->run(acc, gyro);
}

template <typename T>
void EstimationInterface<T>::end_of_cycle_ops()
{
    // maintain positive semidefinite matrix at end - errors can build overtime so just to ensure
    Kalman::StateMatrix<T> tmp = 0.5 * (kf->P + kf->P.transpose());
    kf->P.swap(tmp);

    // delete points outside a range
    if (params<T>.viewing_distance != -1.0)
    {
        // params.box_trim
        Eigen::Matrix<T, 3, 1> curr_pos = state->get_state_info("POS");
        map->delete_outside_points(curr_pos, params<T>.viewing_distance, del_typ);
    }
}

template struct EstimationInterface<double>;
template struct EstimationInterface<float>;