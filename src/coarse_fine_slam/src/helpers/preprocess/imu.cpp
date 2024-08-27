#include "coarse_fine_slam/helpers/preprocess/imu.hpp"

namespace Imu
{
    template <typename T>
    Eigen::Matrix<T, 3, 1> ned_to_enu(const Eigen::Matrix<T, 3, 1> &ned)
    {
        return Eigen::Matrix<T, 3, 1>(ned.y(), ned.x(), -ned.z());
    }

    template <typename T>
    Preprocess<T>::Preprocess() { reset(); }

    template <typename T>
    void Preprocess<T>::reset()
    {
        if (params<T>.imu_en)
            ROS_WARN("[WARN] Reset Imu.");

        iter_num = 1;
        need_init = false;
    }

    template <typename T>
    void Preprocess<T>::convert_n_process(const sensor_msgs::Imu::ConstPtr &msg)
    {
        Eigen::Matrix<T, 3, 1> c_acc, c_gyro;

        c_acc = Eigen::Matrix<T, 3, 1>(
            T(msg->linear_acceleration.x),
            T(msg->linear_acceleration.y),
            T(msg->linear_acceleration.z));

        c_gyro = Eigen::Matrix<T, 3, 1>(
            T(msg->angular_velocity.x),
            T(msg->angular_velocity.y),
            T(msg->angular_velocity.z));

        if (params<T>.imu.is_ned)
        {
            c_acc = ned_to_enu(c_acc);
            c_gyro = ned_to_enu(c_gyro);
        }

        // store into imu bucket
        imu_storage.emplace(std::make_shared<Frame::Imu<T>>(c_acc, c_gyro, T(msg->header.stamp.toSec())));
    }

    template <typename T>
    void Preprocess<T>::filter_data(T curr_timestamp)
    {
        if (pre_data_exist && pre_data->timestamp > curr_timestamp)
            return;

        // clear all imu_data lower than
        bool no_change = true;
        while (imu_storage.try_pop(pre_data))
        {
            if (pre_data->timestamp > curr_timestamp)
            {
                pre_data_exist = true;
                no_change = false;
            }
        }

        if (no_change)
        {
            pre_data = nullptr;
            pre_data_exist = false;
        }
    }

    template <typename T>
    Frame::BasePtr<T> Preprocess<T>::get_next(T lidar_timestamp)
    {

        const double per_init = double(iter_num) / double(params<T>.imu.reset) * 100;
        if (need_init)
        {
            reset();
            ROS_INFO("IMU Initializing: %.1f %%", per_init);
        }

        if (pre_data_exist && pre_data->timestamp < lidar_timestamp)
        {
            pre_data_exist = false;
            return pre_data;
        }

        if (imu_storage.try_pop(pre_data))
        {
            iter_num++;
            if (first_frame)
            {
                params<T>.imu.mean_acc = pre_data->acc;
                params<T>.imu.mean_gyro = pre_data->gyro;
                first_frame = false;
            }
            else // update mean accordingly
            {
                params<T>.imu.mean_acc += (pre_data->acc - params<T>.imu.mean_acc) / iter_num;
                params<T>.imu.mean_gyro += (pre_data->gyro - params<T>.imu.mean_gyro) / iter_num;
            }

            if (params<T>.imu.reset <= int(iter_num))
                need_init = true;

            if (pre_data->timestamp < lidar_timestamp)
                return pre_data;
            else
                pre_data_exist = true;
        }

        return nullptr;
    }

    template <typename T>
    void Preprocess<T>::collect_valid_imu_data(T lidar_start_time, std::vector<Frame::BasePtr<T>> &seq)
    {
        if (need_init)
        {
            T per_init = static_cast<T>(iter_num) / static_cast<T>(params<T>.imu.reset) * 100;
            ROS_INFO("IMU Initializing: %.1f %%", per_init);
            reset();
        }

        if (pre_data_exist)
        {
            if (pre_data->timestamp <= lidar_start_time)
            {
                pre_data_exist = false;
                seq.emplace_back(pre_data);
            }
        }

        while (imu_storage.try_pop(pre_data))
        {
            if (pre_data->timestamp > lidar_start_time)
            {
                pre_data_exist = true;
                break;
            }

            seq.emplace_back(pre_data);

            if (first_frame)
            {
                params<T>.imu.mean_acc = pre_data->acc;
                params<T>.imu.mean_gyro = pre_data->gyro;
                first_frame = false;

                continue;
            }

            // update mean accordingly
            params<T>.imu.mean_acc.noalias() += (pre_data->acc - params<T>.imu.mean_acc) / iter_num;
            params<T>.imu.mean_gyro.noalias() += (pre_data->gyro - params<T>.imu.mean_gyro) / iter_num;
            iter_num++;
        }

        if (params<T>.imu.reset <= int(iter_num))
            need_init = true;
    }

    template struct Preprocess<double>;
    template struct Preprocess<float>;
};