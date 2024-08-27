#include "coarse_fine_slam/state_estimation/helpers/h_modeller.hpp"
#include <tuple>

namespace
{
    template <typename T>
    inline std::tuple<Eigen::Matrix<T, 3, 1>, bool, T> plane_est(Eigen::Matrix<T, Eigen::Dynamic, 3> &A, Eigen::Matrix<T, Eigen::Dynamic, 1> &b)
    {
        Eigen::Matrix<T, 3, 1> norm_vec = A.colPivHouseholderQr().solve(b);

        T n_norm = norm_vec.norm();
        if (n_norm == 0)
            return {Eigen::Matrix<T, 3, 1>::Zero(), false, 0.0};

        norm_vec /= n_norm;
        T d_plane = 1.0 / n_norm;

        for (size_t idx = 0; idx < size_t(A.rows()); ++idx)
        {
            if (std::abs(A.row(idx).dot(norm_vec) + d_plane) > 0.1)
                return {norm_vec, false, d_plane};
        }

        return {norm_vec, true, d_plane};
    }

}

template <typename T>
H_modeller<T>::H_modeller(const Eigen::Matrix<T, 3, 1> &qp, SearchHeap<T> &points)
    : world_coord(qp)
{
    int n = points.size();
    if (n == 0)
    {
        valid = false;
        return;
    }

    Eigen::Matrix<T, Eigen::Dynamic, 3> A(n, 3);
    Eigen::Matrix<T, Eigen::Dynamic, 1> b(n, 1);
    A.setZero();
    b.setOnes();
    b *= -T(1.0);

    T sum = 0;

    size_t idx = 0;
    while (!points.empty())
    {
        A.row(idx) = points.top().second.transpose();
        sum += std::sqrt(points.top().first);
        points.pop();

        if (points.empty())
        {
            matched = A.row(idx).transpose();
            if ((qp - matched).norm() <= T(0.1)) // within 10 cm
                close_proximity = true;
        }

        ++idx;
    }

    T d_plane = 0;
    std::tie(s_norm, valid, d_plane) = plane_est(A, b);
    if (!valid)
        return;

    dist_to_plane = s_norm.transpose() * (qp - matched);
    weight = sum / static_cast<T>(idx + 1);
}

template <typename T>
std::string H_modeller<T>::to_string() const
{
    std::ostringstream oss;
    oss << "H_modeller Attributes:\n";
    oss << "----------------------\n";
    oss << "Matched Point: [" << matched.transpose() << "]\n";
    oss << "Query Point:   [" << world_coord.transpose() << "]\n";
    oss << "Normal Vector: [" << s_norm.transpose() << "]\n";
    oss << "Distance to Plane: " << dist_to_plane << "\n";
    oss << "Valid: " << (valid ? "true" : "false") << "\n";
    oss << "Weight: " << weight << std::endl;
    return oss.str();
}

template <typename T>
void H_modeller<T>::print() const
{
    std::cout << to_string();
}

template struct H_modeller<double>;
template struct H_modeller<float>;
