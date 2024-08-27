#include "coarse_fine_slam/state_estimation/helpers/imu.hpp"

template <typename T>
ImuKf<T>::ImuKf(const Kalman::StatePtr<T> &state, const Kalman::FilterPtr<T> &kf)
    : state(state), kf(kf)
{
    gyro_noise = kf->noise_scale * std::pow(params<T>.imu.gyro_measurement_noise, 2);
    acc_noise = kf->noise_scale * std::pow(params<T>.imu.acc_measurement_noise, 2);
}

template <typename T>
void ImuKf<T>::run(const Eigen::Matrix<T, 3, 1> &acc, const Eigen::Matrix<T, 3, 1> &gyro)
{
    state->clone_state();
    // acc and gyro readings
    acc_m = acc;
    gyro_m = gyro;

    kf->prev_cost.setZero();

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> noise, noise_inv;
    size_t prev_size = 0, row_dim = 0;
    for (size_t idx = 0; idx < params<T>.num_iter; ++idx)
    {
        Tracker<T> sd;

        h_model_build(acc, gyro, sd);
        row_dim = sd.jacob.rows();

        if (prev_size != row_dim)
        {
            noise = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Identity(row_dim, row_dim);
            noise.diagonal() << sd.noise;
            noise_inv = noise.inverse();
            prev_size = row_dim;
        }

        if (kf->step(sd, noise, noise_inv, (idx + 1) == params<T>.num_iter, true))
            break;
    }
}

template <typename T>
void ImuKf<T>::h_model_build(const Eigen::Matrix<T, 3, 1> &acc, const Eigen::Matrix<T, 3, 1> &gyro, Tracker<T> &sd)
{
    Eigen::Matrix<T, 3, 3> bat_val_inv;
    Eigen::Matrix<T, 3, 1> acc_p_ba;
    { // calculating gyro residuals
        sd.z = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(6, 1);
        sd.z.segment(0, 3) = gyro - state->get_state_info("PRED_GYRO") - state->get_state_info("BGA");

        // calculating acc residuals
        Eigen::Matrix<T, 3, 3> bat_val = Eigen::Matrix<T, 3, 3>::Zero();
        bat_val.diagonal() << state->get_state_info("BAT");
        bat_val_inv = bat_val.inverse();

        acc_p_ba = state->get_state_info("BAA") + state->get_state_info("PRED_ACC");
        sd.z.segment(3, 3) = bat_val * acc - acc_p_ba;
    }

    // calculating the jacobian for gyro
    sd.jacob = Eigen::Matrix<T, Eigen::Dynamic, StateRef::TOTAL_DIM>::Zero(6, StateRef::TOTAL_DIM);

    for (const auto &var : {"PRED_GYRO", "BGA"})
    {
        auto var_info = get_var_info(var);
        sd.jacob.block(0, var_info.index, 3, 3).setIdentity();
    }

    // pred acc
    auto var_info = get_var_info("PRED_ACC");
    sd.jacob.block(3, var_info.index, 3, 3) = bat_val_inv;

    // pred baa
    var_info = get_var_info("BAA");
    sd.jacob.block(3, var_info.index, 3, 3) = bat_val_inv;

    // pred bat
    var_info = get_var_info("BAT");
    Eigen::Matrix<T, 3, 3> acc_baa = Eigen::Matrix<T, 3, 3>::Zero();
    acc_baa.diagonal() << acc_p_ba;
    sd.jacob.block(3, var_info.index, 3, 3) = -bat_val_inv * acc_baa * bat_val_inv;

    // noise information and saturation check
    sd.noise = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(6, 1);

    // used to mask saturated imu values.
    sd.imu_satu_mask.setIdentity();

    // check if any axis is saturated we don't update that segment
    for (size_t idx = 0; idx < 3; ++idx)
    {
        // checking gyro axis
        if (std::abs(gyro(idx)) >= 0.99 * params<T>.imu.satu_threshold.gyro)
        {
            sd.z(idx) = 0;
            sd.imu_satu_mask(idx, idx) = 0.0;
        }

        // checking acc axis
        if (std::abs(acc(idx)) >= 0.99 * params<T>.imu.satu_threshold.acc)
        {
            sd.z(idx + 3) = 0;
            sd.imu_satu_mask(idx + 3, idx + 3) = 0.0;
        }

        // fill noise info
        sd.noise(idx) = gyro_noise;
        sd.noise(idx + 3) = acc_noise;
    }
}

template struct ImuKf<double>;
template struct ImuKf<float>;