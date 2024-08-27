#ifndef IMU_ESTIMATION_HPP
#define IMU_ESTIMATION_HPP

#include "coarse_fine_slam/helpers/calc_helpers.hpp"
#include "coarse_fine_slam/state_estimation/helpers/kf.hpp"
#include "parameters.hpp"
#include "state.hpp"

template <typename T>
struct ImuKf
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<ImuKf<T>>;

    ImuKf() = default;

    ImuKf(const Kalman::StatePtr<T> &state, const Kalman::FilterPtr<T> &kf);

    void run(const Eigen::Matrix<T, 3, 1> &acc, const Eigen::Matrix<T, 3, 1> &gyro);

private:
    void h_model_build(const Eigen::Matrix<T, 3, 1> &acc, const Eigen::Matrix<T, 3, 1> &gyro, Tracker<T> &sd);

private: // attributes
    Kalman::StatePtr<T> state;
    Kalman::FilterPtr<T> kf = nullptr;

    // helper functions
    Eigen::Matrix<T, 3, 1> acc_m, gyro_m;
    T acc_noise, gyro_noise;
};

template <typename T>
using ImuKfPtr = typename ImuKf<T>::Ptr;
#endif