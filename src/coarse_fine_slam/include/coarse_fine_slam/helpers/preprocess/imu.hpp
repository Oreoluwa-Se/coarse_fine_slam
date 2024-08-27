#ifndef IMU_PREPROCESS_HPP
#define IMU_PREPROCESS_HPP

#include "parameters.hpp"
#include "merged_data.hpp"
#include <sensor_msgs/Imu.h>
#include <tbb/concurrent_queue.h>

namespace Imu
{

    template <typename T>
    struct Preprocess
    {
        using Ptr = std::shared_ptr<Preprocess<T>>;
        Preprocess();

        void convert_n_process(const sensor_msgs::Imu::ConstPtr &msg);

        void collect_valid_imu_data(T lidar_start_time, std::vector<Frame::BasePtr<T>> &seq);

        Frame::BasePtr<T> get_next(T lidar_timestamp);

        void filter_data(T timestamp);

    private:
        void reset();

    private:
        tbb::concurrent_queue<Frame::ImuPtr<T>> imu_storage;
        bool is_double = std::is_same<T, double>::value;
        bool first_frame = true, need_init = true;
        Frame::ImuPtr<T> pre_data = nullptr;
        bool pre_data_exist = false;
        // T prev_times
        size_t iter_num = 0;
    };

    template <typename T>
    using PreprocessPtr = typename Preprocess<T>::Ptr;
};
#endif