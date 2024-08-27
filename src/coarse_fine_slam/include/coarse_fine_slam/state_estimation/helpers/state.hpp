#ifndef STATE_HPP
#define STATE_HPP

#include "parameters.hpp"
#include "coarse_fine_slam/helpers/calc_helpers.hpp"
#include <map_storage/utils/alias.hpp>
#include <utility>
#include <map>
#include <unordered_map>

/*
 * State variable representation - Also includes info on how we update the state.
 * We update the imu readings as part of the state.
 */
template <typename T>
struct Tracker
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool valid = false, converge = false;
    Eigen::Matrix<T, Eigen::Dynamic, 1> z;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> jacob; // full H-matrix
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> noise;
    Eigen::Matrix<T, 6, 6> imu_satu_mask;

    void clear()
    {
        z.setZero();
        jacob.setZero();
        noise.setZero();
    }
};

namespace StateRef
{
    enum class VarName
    {
        POS,       // imu position in world coordinates
        VEL,       // imu velocity in world coordinates
        ORI,       // imu orientation in world coordinates
        GRAV,      // gravity in world coordinates
        BGA,       // gyro additive bias
        BAA,       // acc additive bias
        BAT,       // acc multiplicative bias
        POS_BL,    // base to lidar translation in world coordinates
        ORI_BL,    // base to lidar orientation in world coordinates
        POS_BI,    // base to imu translation in world coordinates
        ORI_BI,    // base to imu translation in world coordinates
        PRED_GYRO, // predicted rotational velocity of imu.
        PRED_ACC   // predicted linear acceleration of imu.
    };

    enum class PoseType
    {
        // _ means relative so a_b implies a relative fo b
        Lidar_Base,
        Imu_Base,
        Imu_Lidar,
        World_Imu
    };

    enum class AdxType
    {
        World,
        Lidar,
        Imu
    };

    struct VarInfo
    {
        size_t index;
        size_t size;
    };

    struct VarPos
    {
        size_t index_1;
        size_t index_2;
    };

    static const std::map<VarName, VarInfo> var_info_map = {
        {VarName::ORI, {0, 3}},
        {VarName::VEL, {3, 3}},
        {VarName::POS, {6, 3}},
        {VarName::GRAV, {9, 3}},
        {VarName::BGA, {12, 3}},
        {VarName::BAA, {15, 3}},
        {VarName::BAT, {18, 3}},

        // lidar base - can be estimated if online calibration is triggered
        {VarName::ORI_BL, {21, 3}},
        {VarName::POS_BL, {24, 3}},

        // imu base - can be estimated if online calibration is triggered
        {VarName::ORI_BI, {27, 3}},
        {VarName::POS_BI, {30, 3}},

        // imu parameters predicted as part of the state
        {VarName::PRED_GYRO, {33, 3}},
        {VarName::PRED_ACC, {36, 3}}};

    static const std::map<std::string, VarName> name_map = {
        {"POS", VarName::POS},
        {"VEL", VarName::VEL},
        {"ORI", VarName::ORI},
        {"GRAV", VarName::GRAV},
        {"BGA", VarName::BGA},
        {"BAA", VarName::BAA},
        {"BAT", VarName::BAT},
        {"POS_BI", VarName::POS_BI},
        {"ORI_BI", VarName::ORI_BI},
        {"POS_BL", VarName::POS_BL},
        {"ORI_BL", VarName::ORI_BL},
        {"PRED_GYRO", VarName::PRED_GYRO},
        {"PRED_ACC", VarName::PRED_ACC}};

    static const std::unordered_map<std::string, std::string> unit_map = {
        {"VEL", " (m/s)"},
        {"GRAV", " (m/s^2)"},
        {"BGA", " (rad/s)"},
        {"PRED_GYRO", " (rad/s)"},
        {"BAA", " (m/s^2)"},
        {"PRED_ACC", " (m/s^2)"},
        {"POS", " (m)"},
        {"POS_BL", " (m)"},
        {"POS_BI", " (m)"},
        {"ORI", " (rad)"},
        {"ORI_BL", " (rad)"},
        {"ORI_BI", " (rad)"}};

    // total state dimension
    constexpr size_t TOTAL_DIM = 39;

    /*--- Adhoc Parameters ---*/
    // size of jacobian from imu_measurement
    constexpr size_t h_imu_jacob_row = 6;
    constexpr size_t h_imu_jacob_col = 15;
    constexpr size_t h_lidar_jacob_col = 14;

    // Trail parameters
    constexpr size_t POSE_DIM = 6;
    constexpr size_t WORLD_SE2_3_DIM = 9;
    constexpr size_t POINT_DIM = 3;
    constexpr size_t ORI_DIM = 3;
};

namespace
{
    inline StateRef::VarInfo get_var_info(const std::string &part_name)
    {
        auto name_it = StateRef::name_map.find(part_name);
        if (name_it == StateRef::name_map.end())
        {
            throw std::invalid_argument("Invalid part name: " + part_name);
        }

        auto var_info_it = StateRef::var_info_map.find(name_it->second);
        if (var_info_it == StateRef::var_info_map.end())
        {
            throw std::invalid_argument("Part name not found in var_info_map: " + part_name);
        }

        return var_info_it->second;
    }

    inline StateRef::VarPos get_var_pos(const std::string &part_1, const std::string &part_2)
    {
        auto v1 = get_var_info(part_1);
        auto v2 = get_var_info(part_2);

        return {v1.index, v2.index};
    }
}

namespace Kalman
{
    // used to represent the kalman state vector
    template <typename T>
    using StateVector = Eigen::Matrix<T, StateRef::TOTAL_DIM, 1>;

    template <typename T>
    using StateMatrix = Eigen::Matrix<T, StateRef::TOTAL_DIM, StateRef::TOTAL_DIM>;

    template <typename T>
    struct StateTracker
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        StateVector<T> info;

        Eigen::Matrix<T, 3, 3> curr_rot;            // rotation matrix
        Eigen::Matrix<T, 3, 3> curr_imu_base_rot;   // lidar base rotation matrix
        Eigen::Matrix<T, 3, 3> curr_lidar_base_rot; // imu base rotation matrix
    };

    template <typename T>
    struct PredNextstate
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix<T, 3, 3> rot_pred;
        Eigen::Matrix<T, 3, 1> v_pred;
        Eigen::Matrix<T, 3, 1> p_pred;
    };

    template <typename T>
    using ComponentData = std::map<std::string, std::pair<std::vector<T>, std::string>>;

    template <typename T>
    struct State
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<State<T>>;

        State();

        void get_init_sensor_pose();

        void set_init_orientation();

        void set_state(const std::string &part_name, const Eigen::Matrix<T, Eigen::Dynamic, 1> &val);

        Eigen::Matrix<T, Eigen::Dynamic, 1> get_state_info(const std::string &part_name);

        static std::string to_string(const StateVector<T> &state);

        SE3Type<T> get_pose(StateRef::PoseType ptype = StateRef::PoseType::World_Imu);

        void print();

        StateMatrix<T> get_phi(T dt, bool imu_data);

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> SEK3_Adjoint(StateRef::AdxType typ = StateRef::AdxType::World);

        PredNextstate<T> predict_next(T dt);

        void state_prediction(T dt, bool imu_data);

        SE3Type<T> lidar_imu_world();

        void clone_state();

        void state_jacob();

        void delta_increment(const StateVector<T> &inc);

        StateVector<T> get_dx();

        void maintain_prev_state();

        std::string to_yaml();

    private:
        void delta();

        ComponentData<T> gather_state_data();

    public:
        StateTracker<T> cloned;

        Eigen::Matrix<T, 3, 3> curr_rot;            // current rotation matrix
        Eigen::Matrix<T, 3, 3> curr_imu_base_rot;   // current lidar base rotation matrix
        Eigen::Matrix<T, 3, 3> curr_lidar_base_rot; // current imu base rotation matrix
        StateMatrix<T> jac;

    private: // attributes
        StateVector<T> info, dx;
        T noise_scale;
        std::vector<std::string> lin_comps, ori_comps;
    };

    template <typename T>
    using StatePtr = typename State<T>::Ptr;
};

#endif