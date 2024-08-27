#include "coarse_fine_slam/state_estimation/helpers/state.hpp"
#include <sophus/so3.hpp>
#include <yaml-cpp/yaml.h>

namespace Kalman
{
    template <typename T>
    State<T>::State()
    {
        info.setZero();

        get_init_sensor_pose();

        noise_scale = std::pow(params<T>.kf.noise_scale, 2);

        auto var_info = get_var_info("BAT");
        (info.segment(var_info.index, var_info.size) << 1, 1, 1).finished();

        // seperating compnents treated linearly
        lin_comps.insert(lin_comps.end(), {"GRAV", "POS_BL", "POS_BI"});

        if (params<T>.imu_en)
            lin_comps.insert(lin_comps.end(), {"BGA", "BAA", "BAT", "PRED_GYRO", "PRED_ACC"});
    }

    template <typename T>
    void State<T>::get_init_sensor_pose()
    {
        set_state("POS_BI", params<T>.imu.translation);
        set_state("POS_BL", params<T>.lidar.translation);

        // sorting orientation and position information
        curr_imu_base_rot = params<T>.imu.orientation;
        curr_lidar_base_rot = params<T>.lidar.orientation;

        set_state("ORI_BI", PoseFuncs<T>::rmat_to_euler(curr_imu_base_rot));
        set_state("ORI_BL", PoseFuncs<T>::rmat_to_euler(curr_lidar_base_rot));
    }

    template <typename T>
    void State<T>::set_init_orientation()
    {
        // sets the world orientation
        Eigen::Quaternion<T> orient = Eigen::Quaternion<T>::FromTwoVectors(-get_state_info("GRAV"), params<T>.imu.mean_acc);
        Eigen::Matrix<T, 4, 1> q_vec = PoseFuncs<T>::quat_to_vec(orient);
        curr_rot = PoseFuncs<T>::quat_to_rmat(q_vec);

        set_state("ORI", PoseFuncs<T>::rmat_to_euler(curr_rot));
    }

    template <typename T>
    SE3Type<T> State<T>::get_pose(StateRef::PoseType ptype)
    {
        Eigen::Matrix<T, Eigen::Dynamic, 1> pos;

        switch (ptype)
        {
        case StateRef::PoseType::Imu_Base:
        {
            pos = get_state_info("POS_BI");
            return SE3Type<T>(curr_imu_base_rot, pos);
        }

        case StateRef::PoseType::Lidar_Base:
        {
            pos = get_state_info("POS_BL");
            return SE3Type<T>(curr_lidar_base_rot, pos);
        }

        case StateRef::PoseType::World_Imu:
        {
            pos = get_state_info("POS");
            return SE3Type<T>(curr_rot, pos);
        }

        case StateRef::PoseType::Imu_Lidar:
        {
            Eigen::Matrix<T, Eigen::Dynamic, 1> pos_bi = get_state_info("POS_BI");
            SE3Type<T> pi = SE3Type<T>(curr_imu_base_rot, pos_bi);

            Eigen::Matrix<T, Eigen::Dynamic, 1> pos_bl = get_state_info("POS_BL");
            SE3Type<T> pl = SE3Type<T>(curr_lidar_base_rot, pos_bl);

            return pi * pl.inverse();
        }

        default:
            throw std::invalid_argument("Invalid PoseType");
        }
    }

    template <typename T>
    SE3Type<T> State<T>::lidar_imu_world()
    {
        SE3Type<T> imu_lidar = get_pose(StateRef::PoseType::Imu_Lidar);
        SE3Type<T> world_imu = get_pose(StateRef::PoseType::World_Imu);
        return world_imu * imu_lidar;
    }

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> State<T>::get_state_info(const std::string &part_name)
    {
        const auto var_info = get_var_info(part_name);
        return info.segment(var_info.index, var_info.size);
    }

    template <typename T>
    void State<T>::set_state(const std::string &part_name, const Eigen::Matrix<T, Eigen::Dynamic, 1> &val)
    {
        const auto var_info = get_var_info(part_name);
        info.segment(var_info.index, var_info.size) = val;
    }

    template <typename T>
    std::string State<T>::to_string(const StateVector<T> &state)
    {
        auto determine_unit = [](const std::string &part_name) -> std::string
        {
            auto it = StateRef::unit_map.find(part_name);
            if (it != StateRef::unit_map.end())
                return it->second;

            return " ";
        };

        std::vector<std::string> var = {"ORI", "POS", "VEL", "GRAV", "ORI_BL", "POS_BL"};
        if (params<T>.imu_en)
            var.insert(var.end(), {"ORI_BI", "POS_BI", "BGA", "BAA", "BAT", "PRED_GYRO", "PRED_ACC"});

        std::ostringstream stream;
        stream << "State Overview:\n";
        stream << "  * Total State Dimension: " << var.size() << " components\n\n";
        stream << "==========================================================\n";
        stream << "Components:\n";

        for (const auto &part_name : var)
        {
            auto var_info = get_var_info(part_name);
            std::string part_value_str = PrintFuncs<T>::Tvec_to_string(state.segment(var_info.index, var_info.size));
            stream << "  - " << std::setw(10) << part_name << ": " << part_value_str << determine_unit(part_name) << "\n";
        }

        stream << "==========================================================\n";
        return stream.str();
    }

    template <typename T>
    void State<T>::print()
    {
        std::cout << State<T>::to_string(info);
    }

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> State<T>::SEK3_Adjoint(StateRef::AdxType typ)
    {
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> se2_3;
        if (typ == StateRef::AdxType::World)
        {
            const auto var_info = get_var_info("VEL");
            se2_3 = PoseFuncs<T>::vec_to_SEK3(curr_rot, info.segment(var_info.index, 6));
        }
        else if (typ == StateRef::AdxType::Imu)
        {
            auto var_info = get_var_info("POS_BI");
            se2_3 = PoseFuncs<T>::vec_to_SEK3(curr_imu_base_rot, info.segment(var_info.index, 3));
        }
        else if (typ == StateRef::AdxType::Lidar)
        {
            auto var_info = get_var_info("POS_BL");
            se2_3 = PoseFuncs<T>::vec_to_SEK3(curr_lidar_base_rot, info.segment(var_info.index, 3));
        }
        else
            throw std::invalid_argument("Invalid Adjoint derivation Attempt. Either World, Imu, or Lidar Options should be selected");

        return PoseFuncs<T>::adj_SEK3(se2_3);
    }

    template <typename T>
    void State<T>::clone_state()
    {
        // for tracking information - Useful when we Propagate P matrix
        cloned.info = info;
        cloned.curr_rot = curr_rot;
        cloned.curr_imu_base_rot = curr_imu_base_rot;
        cloned.curr_lidar_base_rot = curr_lidar_base_rot;
    }

    template <typename T>
    void State<T>::maintain_prev_state()
    {
        info = cloned.info;
        curr_rot = cloned.curr_rot;
        curr_imu_base_rot = cloned.curr_imu_base_rot;
        curr_lidar_base_rot = cloned.curr_lidar_base_rot;
    }

    template <typename T>
    PredNextstate<T> State<T>::predict_next(T dt)
    {
        PredNextstate<T> ret;

        // new_rotation
        ret.rot_pred = curr_rot * PoseFuncs<T>::exp_SO3(get_state_info("PRED_GYRO"), dt);
        Eigen::Matrix<T, 3, 1> new_acc = curr_rot * get_state_info("PRED_ACC") + get_state_info("GRAV");
        ret.v_pred = get_state_info("VEL") + dt * new_acc;
        ret.p_pred = get_state_info("POS") + get_state_info("VEL") * dt + 0.5 * std::pow(dt, 2) * (new_acc);

        return ret;
    }

    template <typename T>
    void State<T>::state_prediction(T dt, bool imu_data)
    {
        auto nxt = predict_next(dt);

        // updating state vector with new information
        auto var_info = get_var_info("POS");
        info.segment(var_info.index, var_info.size) = nxt.p_pred;

        // update rotation information
        var_info = get_var_info("ORI");
        info.segment(var_info.index, var_info.size) = PoseFuncs<T>::rmat_to_euler(nxt.rot_pred);
        curr_rot = nxt.rot_pred;

        var_info = get_var_info("VEL");
        info.segment(var_info.index, var_info.size) = nxt.v_pred;

        // drift propagation only when we have imu readings
        if (params<T>.imu_en && imu_data)
        {
            // Ornstein–Uhlenbeck Reference: https://arxiv.org/pdf/2106.11857
            if (params<T>.kf.baa_process_noise > 0.0)
            {
                var_info = get_var_info("BAA");
                info.segment(var_info.index, var_info.size) *= std::exp(-dt * params<T>.kf.noise_process_BAARev);
            }

            if (params<T>.kf.bga_process_noise > 0.0)
            {
                var_info = get_var_info("BGA");
                info.segment(var_info.index, var_info.size) *= std::exp(-dt * params<T>.kf.noise_process_BGARev);
            }
        }
    }

    template <typename T>
    StateMatrix<T> State<T>::get_phi(T dt, bool imu_data)
    {
        // Main controller for state evolution.
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A_mat = StateMatrix<T>::Zero();

        // Rotation
        auto var_pos = get_var_pos("ORI", "PRED_GYRO");
        A_mat.block(var_pos.index_1, var_pos.index_2, StateRef::POINT_DIM, StateRef::ORI_DIM) = curr_rot * dt;

        // Velocity
        var_pos = get_var_pos("VEL", "PRED_ACC");
        A_mat.block(var_pos.index_1, var_pos.index_2, StateRef::POINT_DIM, StateRef::POINT_DIM) = curr_rot * dt;

        Eigen::Matrix<T, 3, 1> value = get_state_info("GRAV");
        var_pos = get_var_pos("VEL", "ORI");
        A_mat.block(var_pos.index_1, var_pos.index_2, StateRef::POINT_DIM, StateRef::POINT_DIM) = PoseFuncs<T>::skew_matrix(value) * dt;

        value = get_state_info("VEL");
        var_pos = get_var_pos("VEL", "PRED_GYRO");
        A_mat.block(var_pos.index_1, var_pos.index_2, StateRef::POINT_DIM, StateRef::POINT_DIM) = PoseFuncs<T>::skew_matrix(value) * curr_rot * dt;

        // Position
        var_pos = get_var_pos("POS", "VEL");
        A_mat.block(var_pos.index_1, var_pos.index_2, StateRef::POINT_DIM, StateRef::POINT_DIM).setIdentity() *= dt;

        value = get_state_info("POS");
        var_pos = get_var_pos("POS", "PRED_GYRO");
        A_mat.block(var_pos.index_1, var_pos.index_2, StateRef::POINT_DIM, StateRef::POINT_DIM) = PoseFuncs<T>::skew_matrix(value) * curr_rot * dt;

        // imu based derivation -- only present when processing imu data. Doesn't make sense to propagate imu noise during lidar scans
        if (params<T>.imu_en && imu_data)
        {
            if (params<T>.kf.baa_process_noise > 0.0)
            {
                var_pos = get_var_pos("BAA", "BAA");

                T mult_f = -params<T>.kf.noise_process_BAARev * std::exp(-dt * params<T>.kf.noise_process_BAARev) * dt;
                A_mat.block(var_pos.index_1, var_pos.index_2, StateRef::POINT_DIM, StateRef::POINT_DIM).setIdentity() *= mult_f;

                var_pos = get_var_pos("PRED_ACC", "BAA");
                A_mat.block(var_pos.index_1, var_pos.index_2, StateRef::POINT_DIM, StateRef::POINT_DIM).setIdentity() *= -mult_f;
            }

            if (params<T>.kf.gyro_process_noise > 0.0)
            {
                var_pos = get_var_pos("BGA", "BGA");

                T mult_f = -params<T>.kf.noise_process_BGARev * std::exp(-dt * params<T>.kf.noise_process_BGARev) * dt;
                A_mat.block(var_pos.index_1, var_pos.index_2, StateRef::POINT_DIM, StateRef::POINT_DIM).setIdentity() *= mult_f;

                var_pos = get_var_pos("PRED_GYRO", "BGA");
                A_mat.block(var_pos.index_1, var_pos.index_2, StateRef::POINT_DIM, StateRef::POINT_DIM).setIdentity() *= -mult_f;
            }
        }

        // need to take exp then we get phi
        StateMatrix<T> exp_A_mat = PoseFuncs<T>::exp_matrix(A_mat, 4);

        return exp_A_mat;
    }

    template <typename T>
    StateVector<T> State<T>::get_dx() { return dx; }

    template <typename T>
    void State<T>::delta()
    {
        // compare the current to the predicted state
        dx.setZero();

        // ori, vel, position
        auto vec_info = get_var_info("VEL");

        // Convert current and previous states to SE(3) matrices
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> se3_curr = PoseFuncs<T>::vec_to_SEK3(curr_rot, info.segment(vec_info.index, 6));
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> se3_prev = PoseFuncs<T>::vec_to_SEK3(cloned.curr_rot, cloned.info.segment(vec_info.index, 6));

        // Compute the difference (relative transformation) and take the logarithm to get the delta
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> delta_se3 = PoseFuncs<T>::log_SEK3(se3_curr * PoseFuncs<T>::SEK3_inv(se3_prev));

        // Store the result in the difference vector
        vec_info = get_var_info("ORI");
        dx.segment(vec_info.index, 9) = PoseFuncs<T>::SEK3_to_vec(delta_se3, true);

        // Compute delta for Lidar sensor calibration
        auto v_info = get_var_info("ORI_BL");
        Eigen::Matrix<T, 3, 3> rot_diff = curr_lidar_base_rot * cloned.curr_lidar_base_rot.transpose();
        dx.segment(v_info.index, v_info.size) = Sophus::SO3<T>(rot_diff).log();

        v_info = get_var_info("ORI_BI");
        rot_diff = curr_imu_base_rot * cloned.curr_imu_base_rot.transpose();
        dx.segment(v_info.index, v_info.size) = Sophus::SO3<T>(rot_diff).log();

        // Compute linear differences
        for (const auto &var : lin_comps)
        {
            v_info = get_var_info(var);
            dx.segment(v_info.index, v_info.size) = info.segment(v_info.index, v_info.size) - cloned.info.segment(v_info.index, v_info.size);
        }
    }

    template <typename T>
    void State<T>::state_jacob()
    {
        jac.setZero();
        delta();
        /*
         * The cloned state represents the most recent "predicted" information
         * The state tracks the most recent update. Deriving the error wrt to the state gives the jacobian
         */

        // handling orientation, velocity, position
        auto v_info = get_var_info("ORI");
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> adx_lie = PoseFuncs<T>::adj_SEK3_LIE(dx.segment(v_info.index, 9));
        jac.block(v_info.index, v_info.index, 9, 9) = PoseFuncs<T>::left_jacobian_inverse(adx_lie, 4);

        // Overall idea is this -> left jacobian of skew (delta between current updated and inital predicted pose)
        v_info = get_var_info("ORI_BL");
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> skew_ori = PoseFuncs<T>::skew_matrix(dx.segment(v_info.index, 3));
        jac.block(v_info.index, v_info.index, v_info.size, v_info.size) = PoseFuncs<T>::left_jacobian_inverse(skew_ori, 4);

        v_info = get_var_info("ORI_BI");
        skew_ori = PoseFuncs<T>::skew_matrix(dx.segment(v_info.index, 3));
        jac.block(v_info.index, v_info.index, v_info.size, v_info.size) = PoseFuncs<T>::left_jacobian_inverse(skew_ori, 4);

        // things treated linearly are just identity
        for (const auto &var : lin_comps)
        {
            v_info = get_var_info(var);
            jac.block(v_info.index, v_info.index, v_info.size, v_info.size).setIdentity();
        }
    }

    template <typename T>
    void State<T>::delta_increment(const StateVector<T> &inc)
    {
        // ori, vec, pos
        auto var_info = get_var_info("VEL");
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> se3_prev = PoseFuncs<T>::vec_to_SEK3(cloned.curr_rot, cloned.info.segment(var_info.index, 6));

        var_info = get_var_info("ORI");
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> inc_exp = PoseFuncs<T>::exp_SEK3(inc.segment(var_info.index, 9));

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> new_pose = inc_exp * se3_prev;

        // Update the ori, vel, pos information
        curr_rot = new_pose.template block<3, 3>(0, 0);
        info.segment(var_info.index, 9) = PoseFuncs<T>::SEK3_to_vec(new_pose, false);

        // If online calibration is enabled, apply increments for Lidar and IMU sensor calibration

        var_info = get_var_info("ORI_BL");
        Eigen::Matrix<T, 3, 3> rmat = PoseFuncs<T>::exp_SO3(inc.segment(var_info.index, 3), 1.0);
        curr_lidar_base_rot = rmat * cloned.curr_lidar_base_rot;

        var_info = get_var_info("ORI_BI");
        rmat = PoseFuncs<T>::exp_SO3(inc.segment(var_info.index, 3), 1.0);
        curr_imu_base_rot = rmat * cloned.curr_imu_base_rot;

        for (const auto &var : lin_comps)
        {
            var_info = get_var_info(var);
            info.segment(var_info.index, var_info.size) = cloned.info.segment(var_info.index, var_info.size) + inc.segment(var_info.index, var_info.size);
        }
    }

    template <typename T>
    ComponentData<T> State<T>::gather_state_data()
    {
        auto determine_unit = [](const std::string &part_name) -> std::string
        {
            auto it = StateRef::unit_map.find(part_name);
            if (it != StateRef::unit_map.end())
                return it->second;
            return "";
        };

        std::vector<std::string> var = {"ORI", "POS", "VEL", "GRAV", "ORI_BL", "POS_BL"};
        if (params<T>.imu_en)
            var.insert(var.end(), {"ORI_BI", "POS_BI", "BGA", "BAA", "BAT", "PRED_GYRO", "PRED_ACC"});

        ComponentData<T> data;
        for (const auto &part_name : var)
        {
            auto var_info = get_var_info(part_name);
            std::vector<T> values;
            values.resize(var_info.size);

            for (size_t idx = 0; idx < var_info.size; ++idx)
                values[idx] = info.segment(var_info.index, var_info.size)(idx);

            // structuring the data
            data[part_name] = std::make_pair(values, determine_unit(part_name));
        }

        return data;
    }

    template <typename T>
    std::string State<T>::to_yaml()
    {
        YAML::Emitter out;
        auto data = gather_state_data();

        out << YAML::BeginMap;
        out << YAML::Key << "state_dimension" << YAML::Value << data.size();
        out << YAML::Key << "components" << YAML::Value << YAML::BeginMap;

        for (const auto &entry : data)
        {
            out << YAML::Key << entry.first << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "values" << YAML::Value << YAML::Flow << YAML::BeginSeq;
            for (const auto &value : entry.second.first)
                out << value;
            out << YAML::EndSeq;
            out << YAML::Key << "unit" << YAML::Value << entry.second.second;
            out << YAML::EndMap;
        }

        out << YAML::EndMap;
        out << YAML::EndMap;

        return std::string(out.c_str());
    }

    template struct State<double>;
    template struct State<float>;
};

// template <typename T>
// std::string State<T>::to_json()
// {
//     nlohmann::json json_obj;
//     auto data = gather_state_data();

//     json_obj["state_dimension"] = data.size();

//     for (const auto &entry : data)
//     {
//         json_obj["components"][entry.first]["values"] = entry.second.first;
//         json_obj["components"][entry.first]["unit"] = entry.second.second;
//     }

//     return json_obj.dump(4);
// }
