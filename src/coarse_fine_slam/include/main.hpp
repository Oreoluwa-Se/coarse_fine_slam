#ifndef MAIN_RUN_FILE_HPP
#define MAIN_RUN_FILE_HPP

#include <iostream>
#include "parameters.hpp"
#include "coarse_fine_slam/ros_1/acquire_data.hpp"
#include "coarse_fine_slam/ros_1/visualizer.hpp"
#include "coarse_fine_slam/state_estimation/interface.hpp"
#include <string>

template <typename T>
struct SensorTracker
{
    T curr_timestamp = 0.0;

    // tracks last prediction
    T predict_cov_ts = 0.0;

    size_t info_count = 0;
    size_t lidar_frames = 0;
    size_t imu_frames = 0;

    void reset();

    void print() const;
};

template <typename T>
class CFLIO
{
public:
    CFLIO();

    void run_script();

private:
    void run_imu(Frame::BasePtr<T> &frame);

    void predict_stage(Frame::BasePtr<T> &frame, bool imu_data);

    void run_lidar(Frame::BasePtr<T> frame, bool run_ops);

    void run_with_imu(Frame::BasePtr<T> &frame, bool run_ops);

    void establish_path();

    void store_state_info();

private:
    ros::NodeHandle nh;
    InitHelpers<T> init;

    DataAcquisitionPtr<T> acq = nullptr;
    EstimationInterfacePtr<T> kf_interface = nullptr;
    VisualizerPtr<T> viz = nullptr;

    SensorTracker<T> kf_tracker;
    bool init_frame = false, first_frame = true, first_subframe = true;
    size_t main_frame_track = 0;
    std::string file_path;
};

#endif