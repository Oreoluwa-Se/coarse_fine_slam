#include "coarse_fine_slam/helpers/preprocess/merged_data.hpp"

namespace Frame
{
    template <typename T>
    Imu<T>::Imu(const Eigen::Matrix<T, 3, 1> &acc, const Eigen::Matrix<T, 3, 1> &gyro, T ts)
        : Base<T>(SensorType::Imu, ts), acc(acc), gyro(gyro) {}

    template <typename T>
    typename Imu<T>::Ptr Imu<T>::cast(BasePtr<T> &op)
    {
        if (!op)
            return nullptr;

        if (auto casted = std::dynamic_pointer_cast<Imu<T>>(op))
            return casted;

        return nullptr;
    }

    template struct Imu<double>;
    template struct Imu<float>;

    template <typename T>
    Lidar<T>::Lidar()
        : Base<T>(SensorType::Lidar, -1.0) {}

    template <typename T>
    Lidar<T>::Lidar(const T ts)
        : Base<T>(SensorType::Lidar, ts) {}

    template <typename T>
    Lidar<T>::Lidar(const T ts, const Point3dPtrVectCC<T> &frame)
        : Base<T>(SensorType::Lidar, ts), frame(frame) {}

    template <typename T>
    typename Lidar<T>::Ptr Lidar<T>::cast(BasePtr<T> &op)
    {
        if (!op)
            return nullptr;

        if (auto casted = std::dynamic_pointer_cast<Lidar<T>>(op))
            return casted;

        return nullptr;
    }

    template struct Lidar<double>;
    template struct Lidar<float>;
}