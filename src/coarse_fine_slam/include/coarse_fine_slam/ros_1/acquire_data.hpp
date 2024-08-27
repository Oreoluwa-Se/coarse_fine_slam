#ifndef ACQUIRE_DATA_HPP
#define ACQUIRE_DATA_HPP

#include "coarse_fine_slam/helpers/preprocess/imu.hpp"
#include "coarse_fine_slam/helpers/preprocess/lidar.hpp"
#include "parameters.hpp"
#include <atomic>
#include <memory>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tbb/concurrent_queue.h>
#include <thread>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <set>
#include <mutex>
#include <chrono>

template <typename T>
struct SynchedData
{
    /*
     * Represents synchronized frame for imu lidar data.
     * Order is [imu | lidar] data
     */
    bool valid = false;
    std::vector<Frame::BasePtr<T>> imu_data;
    Lidar::ProcessedFramePtr<T> lidar_data;
    size_t imu_idx = 0;

    Frame::BasePtr<T> imu_sub = nullptr, lidar_sub = nullptr;

    Frame::BasePtr<T> get_next();

    bool empty();
};

/* Data collection class*/
template <typename T>
class DataAcquisition
{
public:
    using Ptr = std::shared_ptr<DataAcquisition<T>>;

    DataAcquisition();

    explicit DataAcquisition(ros::NodeHandle &nh);

    DataAcquisition(const DataAcquisition &) = delete;

    DataAcquisition &operator=(const DataAcquisition &) = delete;

    ~DataAcquisition();

    void imu_cbck_help(const sensor_msgs::Imu::ConstPtr &msg);

    void lidar_cbck_help(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void data_extract();

    bool all_data_acquired();

    void update_time_mark(T curr_time);

    SynchedData<T> data_sync();

    // attributes
    std::string imu_header_id;
    std::string pc_header_id;

private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub, lidar_sub;

    std::thread acquire_thread;
    std::atomic<bool> running{true};

    // lidar frame holder
    tbb::concurrent_queue<Lidar::ProcessedFramePtr<T>> lidar_queue;
    Imu::PreprocessPtr<T> imu_handler = nullptr;

    bool imu_id_check = true;
    bool lidar_id_check = true;

    std::atomic<int> lidar_empty_track{0};
    std::atomic<int> imu_empty_track{0};

    // for tracking the time
    boost::shared_mutex mutex;
    T current_time_mark = 0.0;
};

//
template <typename T>
using DataAcquisitionPtr = typename DataAcquisition<T>::Ptr;
#endif
