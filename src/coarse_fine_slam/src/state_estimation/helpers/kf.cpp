#include "coarse_fine_slam/state_estimation/helpers/kf.hpp"

namespace Kalman
{
    namespace
    {
        template <typename T>
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> p_inv(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matrix)
        {
            Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> cod(matrix);
            // Get the pseudoinverse
            return cod.pseudoInverse();
        }

        template <typename T>
        StateMatrix<T> p_inv(const StateMatrix<T> &matrix)
        {
            Eigen::CompleteOrthogonalDecomposition<StateMatrix<T>> cod(matrix);
            // Get the pseudoinverse
            return cod.pseudoInverse();
        }
    }

    template <typename T>
    Filter<T>::Filter(StatePtr<T> state)
        : state(state), ema_Q(std::make_shared<EMAAveraging<T>>(10))
    {
        noise_scale = std::pow(params<T>.kf.noise_scale, 2);
        setup_P();
        setup_Q(); // inital set here.. then the drift occurs over time
    }

    template <typename T>
    void Filter<T>::setup_Q()
    {
        Q.setZero();

        // noise for sensor location

        auto qc = std::pow(params<T>.kf.ori_s_process_noise, 2);
        Eigen::Matrix<T, 3, 1> ori_noise(qc, qc, qc);

        auto var_info = get_var_pos("ORI_BL", "ORI_BL");
        Q.block(var_info.index_1, var_info.index_2, 3, 3) = PoseFuncs<T>::skew_matrix(ori_noise);

        var_info = get_var_pos("ORI_BI", "ORI_BI");
        Q.block(var_info.index_1, var_info.index_2, 3, 3) = PoseFuncs<T>::skew_matrix(ori_noise);

        qc = std::pow(params<T>.kf.pos_s_process_noise, 2);

        var_info = get_var_pos("POS_BL", "POS_BL");
        Q.block(var_info.index_1, var_info.index_2, 3, 3).setIdentity() *= qc;

        var_info = get_var_pos("POS_BI", "POS_BI");
        Q.block(var_info.index_1, var_info.index_2, 3, 3).setIdentity() *= qc;

        if (params<T>.imu_en)
        { // process noise for estimating the omega
            var_info = get_var_pos("BAA", "BAA");
            Q.block(var_info.index_1, var_info.index_2, 3, 3).setIdentity() *= std::pow(params<T>.kf.acc_process_noise, 2);

            var_info = get_var_pos("BGA", "BGA");
            Q.block(var_info.index_1, var_info.index_2, 3, 3).setIdentity() *= std::pow(params<T>.kf.gyro_process_noise, 2);

            var_info = get_var_pos("PRED_GYRO", "PRED_GYRO");
            qc = std::pow(params<T>.kf.gyro_process_noise, 2);
            Q.block(var_info.index_1, var_info.index_2, 3, 3).setIdentity(3, 3) *= qc;

            // process noise for estimating the linear acceleration
            var_info = get_var_pos("PRED_ACC", "PRED_ACC");
            qc = std::pow(params<T>.kf.acc_process_noise, 2);
            Q.block(var_info.index_1, var_info.index_2, 3, 3).setIdentity(3, 3) *= qc;
        }

        Q *= noise_scale;
    }

    template <typename T>
    void Filter<T>::setup_P()
    {
        P.setZero();

        auto set_block = [&](const std::string &var_name, int dim, T noise)
        {
            StateRef::VarPos loc = get_var_pos(var_name, var_name);
            P.block(loc.index_1, loc.index_2, dim, dim).setIdentity(dim, dim) *= std::pow(noise, 2);
        };

        // noise parameters with world
        set_block("POS", StateRef::POINT_DIM, params<T>.kf.pos_initial_noise);
        set_block("VEL", StateRef::POINT_DIM, params<T>.kf.vel_initial_noise);
        set_block("ORI", StateRef::ORI_DIM, params<T>.kf.ori_initial_noise);
        set_block("GRAV", StateRef::POINT_DIM, params<T>.kf.grav_initial_noise);

        // parameters for online calibration

        set_block("POS_BL", StateRef::POINT_DIM, params<T>.kf.pos_s_initial_noise);
        set_block("ORI_BL", StateRef::ORI_DIM, params<T>.kf.ori_s_initial_noise);
        set_block("POS_BI", StateRef::POINT_DIM, params<T>.kf.pos_s_initial_noise);
        set_block("ORI_BI", StateRef::ORI_DIM, params<T>.kf.ori_s_initial_noise);

        // parameters for bias
        if (params<T>.imu_en)
        {
            set_block("BGA", StateRef::POINT_DIM, params<T>.kf.bga_initial_noise);
            set_block("BAA", StateRef::POINT_DIM, params<T>.kf.baa_initial_noise);
            set_block("BAT", StateRef::POINT_DIM, params<T>.kf.bat_initial_noise);
            set_block("PRED_GYRO", StateRef::POINT_DIM, params<T>.kf.pred_gyro_initial_noise);
            set_block("PRED_ACC", StateRef::POINT_DIM, params<T>.kf.pred_acc_initial_noise);
        }

        // Scale and ensure invertibility
        P *= noise_scale;
    }

    // Function to get a 3x3 block from the covariance matrix P
    template <typename T>
    Eigen::Matrix<T, 3, 3> Filter<T>::get_P_block(const std::string &var1, const std::string &var2)
    {
        StateRef::VarPos loc = get_var_pos(var1, var2);
        return P.block(loc.index_1, loc.index_2, 3, 3);
    }

    template <typename T>
    void Filter<T>::imu_meas_drift(T dt)
    {
        // Noise propagation - accounts for drift that can occur with imu measurments
        if (params<T>.kf.baa_process_noise > 0.0)
        {
            auto var_info = get_var_pos("BAA", "BAA");

            // Adding acc drift to Q matrix
            Eigen::Matrix<T, 3, 3> qc_block = noise_scale * std::pow(params<T>.kf.acc_process_noise, 2) * params<T>.id;
            const T theta = params<T>.kf.noise_process_BAARev;
            if (theta > 0.0)
                qc_block *= ((1 - exp(-2 * dt * theta)) / (2 * theta));

            Q.block(var_info.index_1, var_info.index_2, 3, 3) += qc_block;
        }

        if (params<T>.kf.gyro_process_noise > 0.0)
        {
            auto var_info = get_var_pos("BGA", "BGA");

            // Adding gyro drift to Q matrix
            Eigen::Matrix<T, 3, 3> qc_block = noise_scale * std::pow(params<T>.kf.gyro_process_noise, 2) * params<T>.id;
            const T theta = params<T>.kf.noise_process_BGARev;
            if (theta > 0.0)
                qc_block *= ((1 - exp(-2 * dt * theta)) / (2 * theta));

            Q.block(var_info.index_1, var_info.index_2, 3, 3) += qc_block;
        }
    }

    template <typename T>
    StateMatrix<T> Filter<T>::generate_Q(StateMatrix<T> &phi, T dt, bool imu_data)
    {
        ADx_phi.setIdentity();

        // No noise in position,, velocity, orientation - based on transition equations
        StateRef::VarInfo spi = get_var_info("ORI");
        ADx_phi.template block<9, 9>(spi.index, spi.index) = state->SEK3_Adjoint(StateRef::AdxType::World);

        // The noise drift that occurs when we have imu data
        if (params<T>.imu_en && imu_data)
            imu_meas_drift(dt);

        ADx_phi = ADx_phi * phi;
        return ADx_phi * Q * ADx_phi.transpose() * dt;
    }

    template <typename T>
    bool Filter<T>::step(Tracker<T> &sd, const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &noise, const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &noise_inv, bool final_step, bool is_imu)
    {
        StateMatrix<T> J_inv, P_Jinv;
        Eigen::Matrix<T, Eigen::Dynamic, StateRef::TOTAL_DIM> H_J_inv;
        Eigen::Matrix<T, StateRef::TOTAL_DIM, Eigen::Dynamic> K;
        Eigen::Matrix<T, StateRef::TOTAL_DIM, 1> dx;

        state->state_jacob();
        // J_inv = state->jac.inverse();
        J_inv = p_inv(state->jac);
        {

            if (!is_imu)
                P_Jinv = P * J_inv.transpose();
            else
            {
                // zero out saturated regions
                P_Jinv = P;
                auto info = get_var_info("BGA");

                P_Jinv.block(info.index, info.index, 6, 6) = P_Jinv.block(info.index, info.index, 6, 6) * sd.imu_satu_mask;
                P_Jinv = P_Jinv * J_inv.transpose();
            }

            Eigen::Matrix<T, StateRef::TOTAL_DIM, Eigen::Dynamic> P_Jinv_H_t = P_Jinv * sd.jacob.transpose();
            H_J_inv = sd.jacob * J_inv;

            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> S = H_J_inv * P_Jinv_H_t + noise;

            // Compute Kalman gain
            K = P_Jinv_H_t * S.inverse();

            dx = state->get_dx();
            Eigen::Matrix<T, StateRef::TOTAL_DIM, 1> inc = K * (sd.z + sd.jacob * dx);
            // if (!is_imu)
            // {
            //     std::cout << "\nP " << std::endl;
            //     std::cout << P << std::endl;

            //     std::cout << "\nH model" << std::endl;
            //     std::cout << sd.jacob << std::endl;

            //     std::cout << "\nState Jacobian" << std::endl;
            //     std::cout << state->jac << std::endl;

            //     std::cout << "\nState Jacobian inverse" << std::endl;
            //     std::cout << J_inv << std::endl;

            //     std::cout << "\nH_J_inv" << std::endl;
            //     std::cout << H_J_inv << std::endl;

            //     std::cout << "\nP_Jinv_H_t" << std::endl;
            //     std::cout << P_Jinv_H_t << std::endl;

            //     std::cout << "\nLeft side of S" << std::endl;
            //     std::cout << (H_J_inv * P_Jinv_H_t) << std::endl;

            //     std::cout << "\nS_inverse" << std::endl;
            //     std::cout << S.inverse() << std::endl;

            //     std::cout << "\nDX " << std::endl;
            //     std::cout << dx << std::endl;

            //     std::cout << "\ninc" << std::endl;
            //     std::cout << inc << std::endl;
            //     // std::cout << "\nsd.z" << std::endl;
            //     // std::cout << sd.z << std::endl;
            // }
            // increment the state
            state->delta_increment(inc);
        }

        StateMatrix<T> J_t_P_J = state->jac.transpose() * P.inverse() * state->jac;
        Eigen::Matrix<T, 1, 1> cost = dx.transpose() * p_inv(J_t_P_J) * dx + sd.z.transpose() * noise_inv * sd.z;
        sd.converge = std::abs(cost(0, 0) - prev_cost(0, 0)) <= static_cast<T>(0.001);

        if (final_step || sd.converge)
        {
            P = J_inv * (StateMatrix<T>::Identity() - K * H_J_inv) * P_Jinv;
            return true;
        }

        // update the previous cost
        prev_cost = cost;
        return false;
    }

    template <typename T>
    void Filter<T>::process(T dt, bool predict, bool propagate, bool imu_data)
    {
        if (predict) // update state
            state->state_prediction(dt, imu_data);

        if (propagate)
        {
            // Linearized error dynamics
            phi = state->get_phi(dt, imu_data);
            curr_Q = generate_Q(phi, dt, imu_data);

            P = phi * P * phi.transpose() + ema_Q->updated_matrix(curr_Q);
        }
    }

    template struct Filter<double>;
    template struct Filter<float>;
};