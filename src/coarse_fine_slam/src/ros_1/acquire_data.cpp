#include "coarse_fine_slam/ros_1/acquire_data.hpp"
#include <tbb/parallel_invoke.h>
#include <chrono>
#include <iomanip>

template <typename T>
Frame::BasePtr<T> SynchedData<T>::get_next()
{
    if (!lidar_sub) // fetch next lidar data
        lidar_sub = lidar_data->get_next();

    if (!imu_sub && imu_idx < imu_data.size())
    { // fetch next imu data
        imu_sub = imu_data[imu_idx];
        ++imu_idx;
    }

    Frame::BasePtr<T> data = nullptr;
    if (!imu_sub && lidar_sub)
    {
        data = lidar_sub;
        lidar_sub = nullptr;
    }
    else if (imu_sub && !lidar_sub)
    {
        data = imu_sub;
        imu_sub = nullptr;
    }
    else if ((imu_sub && lidar_sub) && (imu_sub->timestamp < lidar_sub->timestamp))
    {
        data = imu_sub;
        imu_sub = nullptr;
    }
    else
    {
        data = lidar_sub;
        lidar_sub = nullptr;
    }

    return data;
}

template <typename T>
bool SynchedData<T>::empty() { return lidar_data->empty() && imu_idx == imu_data.size(); }

template struct SynchedData<double>;
template struct SynchedData<float>;

template <typename T>
DataAcquisition<T>::DataAcquisition()
    : nh(nh), imu_handler(std::make_shared<Imu::Preprocess<T>>())
{
    // imu subscription
    if (params<T>.imu_en)
        imu_sub = nh.subscribe(params<T>.imu.topic, params<T>.imu.max_queue, &DataAcquisition::imu_cbck_help, this);

    // lidar subscription
    lidar_sub = nh.subscribe(params<T>.lidar.topic, params<T>.lidar.max_queue, &DataAcquisition::lidar_cbck_help, this);
}

template <typename T>
DataAcquisition<T>::DataAcquisition(ros::NodeHandle &nh)
    : nh(nh), imu_handler(std::make_shared<Imu::Preprocess<T>>())
{
    // imu subscription
    if (params<T>.imu_en)
        imu_sub = nh.subscribe(params<T>.imu.topic, params<T>.imu.max_queue, &DataAcquisition::imu_cbck_help, this);

    // lidar subscription
    lidar_sub = nh.subscribe(params<T>.lidar.topic, params<T>.lidar.max_queue, &DataAcquisition::lidar_cbck_help, this);

    // run synchronization thread in parallel
    // acquire_thread = std::thread(&DataAcquisition::data_extract, this);
}

template <typename T>
DataAcquisition<T>::~DataAcquisition()
{
    running = false;
    if (acquire_thread.joinable())
        acquire_thread.join();
}

template <typename T>
void DataAcquisition<T>::imu_cbck_help(const sensor_msgs::Imu::ConstPtr &msg)
{

    if (imu_id_check) // imu_header
    {
        imu_header_id = msg->header.frame_id;
        imu_id_check = false;
    }

    // handles pre-processing and frame conversion
    if (msg)
    {
        {
            boost::shared_lock<boost::shared_mutex> lock(mutex);
            if (T(msg->header.stamp.toSec()) < current_time_mark)
            {
                ++imu_empty_track;
                return;
            }
        }

        imu_handler->convert_n_process(msg);
        imu_empty_track = 0;
    }
    else
        ++imu_empty_track;
}

template <typename T>
void DataAcquisition<T>::update_time_mark(T curr_time)
{
    boost::unique_lock<boost::shared_mutex> lock(mutex);
    current_time_mark = curr_time;
}

template <typename T>
void DataAcquisition<T>::lidar_cbck_help(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (lidar_id_check) // imu_header
    {
        pc_header_id = msg->header.frame_id;
        lidar_id_check = false;
    }

    if (msg)
    {
        update_time_mark(msg->header.stamp.toSec());

        lidar_queue.push(Lidar::Preprocess<T>::process_frame(msg));
        lidar_empty_track = 0;
    }
    else
        ++lidar_empty_track;
}

template <typename T>
void DataAcquisition<T>::data_extract()
{
    // every expected lidar cycle we try to get all [imu frame | lidar frame]
    ros::Rate rate(params<T>.imu.freq);
    while (ros::ok() && running)
    {
        ros::spinOnce();

        // Check if all data has been acquired
        if (all_data_acquired())
        {
            running = false;
            break;
        }

        rate.sleep();
    }
}

template <typename T>
bool DataAcquisition<T>::all_data_acquired()
{
    bool lidar_check = lidar_empty_track.load(std::memory_order_relaxed) > 5;
    bool imu_check = imu_empty_track.load(std::memory_order_relaxed) > 100;

    // Example condition: No new messages received in the last 5 seconds
    return lidar_check && imu_check;
}

template <typename T>
SynchedData<T> DataAcquisition<T>::data_sync()
{
    SynchedData<T> out;

    if (!lidar_queue.try_pop(out.lidar_data))
        return out;

    out.valid = true;
    out.lidar_data->populate_seq();

    if (params<T>.imu_en)
        imu_handler->collect_valid_imu_data(out.lidar_data->check_timestamp, out.imu_data);

    return out;
}

template class DataAcquisition<double>;
template class DataAcquisition<float>;