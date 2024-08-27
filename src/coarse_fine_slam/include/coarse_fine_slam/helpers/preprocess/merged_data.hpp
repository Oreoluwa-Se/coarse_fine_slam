#ifndef MERGED_DATA_HPP
#define MERGED_DATA_HPP

#include "parameters.hpp"
#include <tbb/concurrent_priority_queue.h>

enum struct SensorType
{
    Lidar,
    Imu
};
// ------------ Base For Sensor Types -----------------
namespace Frame
{
    template <typename T>
    struct Base
    {
        using Ptr = std::shared_ptr<Base<T>>;
        SensorType type;
        T timestamp;

        Base() = default;

        Base(SensorType s_type, T timestamp) : type(s_type), timestamp(timestamp) {}

        virtual ~Base() = default;
    };

    template <typename T>
    using BasePtr = typename Base<T>::Ptr;

    // ------------- Imu Sensor ------------
    template <typename T>
    struct Imu : public Base<T>
    {
        using Ptr = std::shared_ptr<Imu<T>>;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix<T, 3, 1> acc;
        Eigen::Matrix<T, 3, 1> gyro;

        Imu(const Eigen::Matrix<T, 3, 1> &acc, const Eigen::Matrix<T, 3, 1> &gyro, T ts);

        static Ptr cast(BasePtr<T> &op);
    };

    template <typename T>
    using ImuPtr = typename Imu<T>::Ptr;

    template <typename T>
    struct Lidar : public Base<T>
    {
        using Ptr = std::shared_ptr<Lidar<T>>;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Point3dPtrVectCC<T> frame;

        Lidar();

        explicit Lidar(const T timestamp);

        Lidar(const T timestamp, const Point3dPtrVectCC<T> &frame);

        static Ptr cast(BasePtr<T> &op);
    };

    template <typename T>
    using LidarPtr = typename Lidar<T>::Ptr;

    // creating priorty_queue:
    template <typename T>
    struct BaseTimeComp
    {
        bool operator()(const BasePtr<T> &lhs, const BasePtr<T> &rhs) const
        {
            return lhs->timestamp > rhs->timestamp; // Greater timestamp means lower priority
        }
    };

    template <typename T>
    // using SequenceQueue = tbb::concurrent_priority_queue<BasePtr<T>, BaseTimeComp<T>>;
    using SequenceQueue = tbb::concurrent_queue<BasePtr<T>>;
};
#endif