#ifndef STATE_ESTIMATION_INTERFACE_HPP
#define STATE_ESTIMATION_INTERFACE_HPP

#include "coarse_fine_slam/helpers/preprocess/merged_data.hpp"
#include "coarse_fine_slam/state_estimation/helpers/kf.hpp"
#include "helpers/state.hpp"
#include "helpers/lidar.hpp"
#include "helpers/imu.hpp"
#include "parameters.hpp"
#include <map_storage/sections/tree_manager.hpp>
#include <Eigen/Dense>

template <typename T>
struct EndofCycleResults
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EndofCycleResults() { reset(); };

    // Member variables
    SE3Type<T> curr_pose;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> pose_cov;
    AVector3TVec<T> frame_points;
    Point3dWPtrVecCC<T> pts;

    // Reset function to clear and reinitialize members
    void reset()
    {
        frame_points.clear();
        pts.clear();
        pose_cov = Eigen::Matrix<T, 6, 6>::Zero();
        curr_pose = SE3Type<T>();
    }
};

template <typename T>
struct EstimationInterface
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<EstimationInterface<T>>;

    EstimationInterface();

    void init_orientation();

    bool build_init();

    void update_base_timestamp(T bt);

    void lidar_frame_stack(Frame::LidarPtr<T> &frame);

    bool lidar_run(bool visualize_map);

    void imu_run(const Eigen::Matrix<T, 3, 1> &acc, const Eigen::Matrix<T, 3, 1> &gyro);

    void end_of_cycle_ops();

    T get_base_timestamp();

public: // attributes
    PointStoragePtr<T> map = nullptr;
    Kalman::StatePtr<T> state = nullptr;
    Kalman::FilterPtr<T> kf = nullptr;
    LidarKfPtr<T> lkf = nullptr;
    ImuKfPtr<T> ikf = nullptr;
    EndofCycleResults<T> cycle_tracker;

private:
    bool orientation_init = false, store_res = false, is_txt = false;
    Point3dPtrVectCC<T> frame_points;
    T base_timestamp = 0.0;
    DeleteType del_typ;
};

template <typename T>
using EstimationInterfacePtr = typename EstimationInterface<T>::Ptr;
#endif