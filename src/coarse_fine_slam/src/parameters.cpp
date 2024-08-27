#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "parameters.hpp"
#include <string>

template <>
Params<double> params<double>;

template <>
Params<float> params<float>;

template <typename T>
void InitHelpers<T>::vect_to_rot(std::vector<T> &vec, Eigen::Matrix<T, 3, 3> &rot)
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rot(i, j) = vec[i * 3 + j];

    // projecting the rotation matrix
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(rot, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<T, 3, 3> normalized_R = svd.matrixU() * svd.matrixV().transpose();
    rot = normalized_R;
}

template <typename T>
void InitHelpers<T>::init_params(const ros::NodeHandle &nh, Params<T> &params)
{
    params.id.setIdentity();

    nh.param<bool>("imu_en", params.imu_en, true);
    nh.param<T>("gravity", params.gravity, 9.81);

    int inter_value;
    nh.param<int>("run_iterations", inter_value, 0);
    params.max_runs = size_t(inter_value);

    nh.param<bool>("verbose", params.verbose, false);
    nh.param<bool>("store_state", params.store_state, false);

    nh.param<std::string>("odom_frame", params.odom_frame, std::string("odom"));
    nh.param<std::string>("base_frame", params.base_frame, std::string(""));
    nh.param<std::string>("result_folder", params.result_folder, std::string(""));

    // Lidar parameters
    nh.param<std::string>("Sensor/Lidar/topic", params.lidar.topic, std::string("/rslidar_points"));
    nh.param<int>("Sensor/Lidar/max_queue", params.lidar.max_queue, 100);

    std::vector<T> lidar_orientation;
    nh.getParam("Sensor/Lidar/orientation", lidar_orientation);
    vect_to_rot(lidar_orientation, params.lidar.orientation);

    std::vector<T> lidar_translation;
    nh.getParam("Sensor/Lidar/translation", lidar_translation);
    params.lidar.translation = Eigen::Matrix<T, 3, 1>(lidar_translation[0], lidar_translation[1], lidar_translation[2]);

    nh.param<T>("Sensor/Lidar/freq", params.lidar.freq, 10.0);
    nh.param<T>("Sensor/Lidar/max_range", params.lidar.max_range, 150.0);
    nh.param<T>("Sensor/Lidar/min_range", params.lidar.min_range, 0.0);
    nh.param<T>("Sensor/Lidar/horizontal_fov", params.lidar.horizontal_fov, 180.0);
    nh.param<T>("Sensor/Lidar/vertical_fov", params.lidar.vertical_fov, 30.0);
    nh.param<int>("Sensor/Lidar/num_scan_lines", params.lidar.num_scan_lines, 16);
    nh.param<T>("Sensor/Lidar/downsample_ratio", params.lidar.downsample_ratio, 0.5);
    nh.param<T>("Sensor/Lidar/measurement_noise", params.lidar.measurement_noise, 0.316);
    nh.param<T>("StateEstimation/search_radius", params.lidar.search_range, 5.0);
    nh.param<T>("StateEstimation/delete_range", params.lidar.delete_range, 1.0);
    nh.param<bool>("StateEstimation/online_calibration", params.online_calib, false);

    nh.param<int>("StateEstimation/num_nearest", inter_value, 5);
    params.lidar.num_nearest = size_t(inter_value);

    nh.param<int>("StateEstimation/num_kf_iterations", inter_value, 2);
    params.num_iter = size_t(inter_value);

    // IMU parameters
    nh.param<std::string>("Sensor/Imu/topic", params.imu.topic, std::string("imu_ned/data"));
    nh.param<int>("Sensor/Imu/max_queue", params.imu.max_queue, 1000);
    nh.param<bool>("Sensor/Imu/is_ned", params.imu.is_ned, false);

    if (params.imu_en)
    {
        std::vector<T> imu_orientation;
        nh.getParam("Sensor/Imu/orientation", imu_orientation);
        vect_to_rot(imu_orientation, params.imu.orientation);

        std::vector<T> imu_translation;
        nh.getParam("Sensor/Imu/translation", imu_translation);
        params.imu.translation = Eigen::Matrix<T, 3, 1>(imu_translation[0], imu_translation[1], imu_translation[2]);

        nh.param<T>("Sensor/Imu/freq", params.imu.freq, 1000.0);
        nh.param<int>("Sensor/Imu/reset", params.imu.reset, 100.0);

        nh.param<T>("Sensor/Imu/satu_threshold/gyro", params.imu.satu_threshold.gyro, 35.0);
        nh.param<T>("Sensor/Imu/satu_threshold/acc", params.imu.satu_threshold.acc, 30.0);
        nh.param<T>("Sensor/Imu/acc_measurement_noise", params.imu.acc_measurement_noise, 0.316);
        nh.param<T>("Sensor/Imu/gyro_measurement_noise", params.imu.gyro_measurement_noise, 0.316);
    }
    else
    {
        // imu and lidar in same location
        params.imu.translation = params.lidar.translation;
        params.imu.orientation = params.lidar.orientation;
        params.imu.freq = 100.0;
    }

    // Build parameters
    nh.param<int>("MapManager/max_points_in_vox", params.build.max_points_in_vox, -1);
    nh.param<int>("MapManager/max_points_in_oct_layer", inter_value, 30);
    params.build.max_points_in_oct_layer = size_t(inter_value);

    nh.param<T>("MapManager/imbal_factor", params.build.imbal_factor, 0.7);
    nh.param<T>("MapManager/del_nodes_factor", params.build.del_nodes_factor, 0.5);
    nh.param<T>("MapManager/viewing_distance", params.viewing_distance, -1.0);
    nh.param<bool>("MapManager/box_trim", params.box_trim, false);
    nh.param<bool>("MapManager/tight_budget", params.tight_budget, true);
    nh.param<bool>("MapManager/visualize_map", params.visualize_map, false);
    nh.param<bool>("MapManager/visualize_subframes", params.visualize_subframes, false);

    nh.param<int>("MapManager/visualize_map_every_nth", inter_value, 30);
    params.full_map_interval = size_t(inter_value);

    params.build.track_stats = false;
    params.build.init_map_size = 100; // not used
    params.build.voxel_size = 1.0;
}

// definging structs
template struct InitHelpers<double>;
template struct InitHelpers<float>;
