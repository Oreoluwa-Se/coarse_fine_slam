#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "coarse_fine_slam/helpers/calc_helpers.hpp"
#include "parameters.hpp"
#include "state.hpp"

namespace Kalman
{
    template <typename T>
    struct Filter
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<Filter<T>>;

        Filter(StatePtr<T> state = nullptr);

        void process(T dt, bool predict = false, bool propagate = false, bool imu_data = false);

        StateMatrix<T> generate_Q(StateMatrix<T> &phi, T dt, bool imu_data = false);

        Eigen::Matrix<T, 3, 3> get_P_block(const std::string &var1, const std::string &var2);

        bool step(Tracker<T> &sd, const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &noise, const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &noise_inv, bool final_step, bool is_imu);

    private:
        void setup_P();

        void setup_Q();

        void imu_meas_drift(T dt);

    public:
        StateMatrix<T> P, Q, phi, curr_Q, ADx_phi;
        StatePtr<T> state = nullptr;
        EMAPtr<T> ema_Q = nullptr;
        Eigen::Matrix<T, 1, 1> prev_cost;
        T noise_scale;
    };

    template <typename T>
    using FilterPtr = typename Filter<T>::Ptr;
};

#endif