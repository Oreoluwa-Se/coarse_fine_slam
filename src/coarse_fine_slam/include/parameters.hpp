#ifndef SYSTEM_PARAMS_HPP
#define SYSTEM_PARAMS_HPP

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <map_storage/utils/alias.hpp>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <vector>

template <typename T>
struct LidarParams
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string topic;
    int max_queue;
    Eigen::Matrix<T, 3, 3> orientation;
    Eigen::Matrix<T, 3, 1> translation;
    T freq;
    T max_range;
    T min_range;
    T horizontal_fov;
    T vertical_fov;
    int num_scan_lines;
    T downsample_ratio;
    T measurement_noise;

    // nearest neighbour search params
    T search_range;
    T delete_range;
    size_t num_nearest;

    // convergence criteria
    T esti = 0.001;
};

template <typename T>
struct ImuParams
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string topic;
    int max_queue;
    bool is_ned;
    Eigen::Matrix<T, 3, 3> orientation;
    Eigen::Matrix<T, 3, 1> translation;
    T freq;
    int reset;

    struct
    {
        T gyro;
        T acc;
    } satu_threshold;
    T acc_measurement_noise;
    T gyro_measurement_noise;

    // used for tracking mean_acc and mean_gyro
    Eigen::Matrix<T, 3, 1> mean_acc = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> mean_gyro = Eigen::Matrix<T, 3, 1>::Zero();
};

template <typename T>
struct BuildParams
{
    int max_points_in_vox;
    size_t max_points_in_oct_layer;
    T imbal_factor;
    T del_nodes_factor;
    bool track_stats;
    int init_map_size;
    T voxel_size;
};

template <typename T>
struct KalmanParams
{
    /*
     * Values here represent standard deviations.
     * Predicted vaules get high uncertainity
     * measured values or values known at begining get low uncertainty
     */

    //- These noise parameters are for position of imu related to world.
    T pos_initial_noise = 0.1;
    T ori_initial_noise = 0.1;
    T vel_initial_noise = 0.1;

    //- These are used for online calibration because we have an initial idea or position they can be small to allow for deviation
    T pos_s_initial_noise = pos_initial_noise / 10;
    T ori_s_initial_noise = ori_initial_noise / 10;
    T pos_s_process_noise = pos_initial_noise / 100;
    T ori_s_process_noise = ori_initial_noise / 100;

    /*
     * Predicted imu acc and gyro parameters
     */
    T pred_acc_initial_noise = 0.1;
    T pred_gyro_initial_noise = 0.1;

    /*
     * Bias Noise Parameters.
     * Modeled similar to https://arxiv.org/pdf/2106.11857
     */
    T bga_initial_noise = 0.1;
    T baa_initial_noise = 0.1;
    T bat_initial_noise = 0.1;
    T grav_initial_noise = 0.1;

    //- Estimated process noise
    T acc_process_noise = 0.03;
    T gyro_process_noise = 0.00017;
    T baa_process_noise = 1e-4;
    T bga_process_noise = 1e-5;
    T bat_process_noise = 1e-3;

    //- Bias Random Walk Modelling Parameters.
    T noise_process_BAARev = 0.1;
    T noise_process_BGARev = 0.1;

    // - used to make sensor more realistic
    T noise_scale = 100;
};

template <typename T>
struct Params
{
    bool imu_en, visualize_map, visualize_subframes, verbose, tight_budget;
    bool store_state;
    std::string odom_frame;
    std::string base_frame;
    std::string result_folder;
    size_t full_map_interval;
    T viewing_distance;
    bool online_calib;
    size_t max_runs; // algorithm runs usually for testing
    size_t num_iter; // kalman filter iterations
    bool box_trim;
    T gravity;

    Eigen::Matrix<T, 3, 3> id;

    LidarParams<T> lidar;
    ImuParams<T> imu;
    BuildParams<T> build;
    KalmanParams<T> kf;
};

template <typename T>
extern Params<T> params;

template <typename T>
struct InitHelpers
{
    static void init_params(const ros::NodeHandle &nh, Params<T> &params);

    static void vect_to_rot(std::vector<T> &vec, Eigen::Matrix<T, 3, 3> &rot);
};

#endif