#ifndef H_MODELLER_HPP
#define H_MODELLER_HPP

#include <map_storage/sections/tree_manager.hpp>
#include "parameters.hpp"
#include <limits>

template <typename T>
struct H_modeller
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    H_modeller() = default;

    H_modeller(const Eigen::Matrix<T, 3, 1> &qp, SearchHeap<T> &points);

    std::string to_string() const;

    void print() const;

    Eigen::Matrix<T, 3, 1> matched = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> world_coord = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> imu_coord = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> src_coord = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> s_norm = Eigen::Matrix<T, 3, 1>::Zero();

    // eigen matrix
    T weight = std::numeric_limits<T>::max();

    size_t vec_loc = 0;
    T dist_to_plane = 0;
    bool valid = false;
    bool close_proximity = false;
};

#endif