#include "coarse_fine_slam/helpers/calc_helpers.hpp"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include "parameters.hpp"
#include <stdexcept>
#include <algorithm>

namespace
{
    template <typename T>
    T clamp(T value, T min_value, T max_value)
    {
        if (value < min_value)
            return min_value;
        if (value > max_value)
            return max_value;
        return value;
    }
}

template <typename T>
T RandomFuncs<T>::quick_select(std::vector<T> &data, size_t k)
{
    size_t left = 0;
    size_t right = data.size() - 1;

    while (left < right)
    {
        size_t piv_idx = left + (right - left) / 2;
        T piv_val = data[piv_idx];
        std::swap(data[piv_idx], data[right]);
        size_t storeIndex = left;

        for (size_t i = left; i < right; ++i)
        {
            if (data[i] < piv_val)
            {
                std::swap(data[i], data[storeIndex]);
                ++storeIndex;
            }
        }
        std::swap(data[storeIndex], data[right]);

        if (storeIndex == k)
            return data[storeIndex];

        else if (storeIndex < k)
            left = storeIndex + 1;

        else
            right = storeIndex - 1;
    }

    return data[left];
}

template <typename T>
std::pair<T, T> RandomFuncs<T>::calculate_quartiles(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matrix, bool lower_only)
{
    // Flatten the matrix into a vector
    std::vector<T> data(matrix.data(), matrix.data() + matrix.size());

    // Ensure the matrix is not empty
    if (data.empty())
        throw std::invalid_argument("The matrix is empty");

    // Calculate positions for the quartiles
    size_t n = data.size();
    T lower_quartile = 0, upper_quartile = 0;
    if (lower_only)
    {
        size_t lower_pos = static_cast<size_t>((n - 1) * 0.25);
        lower_quartile = RandomFuncs<T>::quick_select(data, lower_pos);
        return {lower_quartile, -1.0};
    }

    size_t upper_pos = static_cast<size_t>((n - 1) * 0.75);
    upper_quartile = RandomFuncs<T>::quick_select(data, upper_pos);

    return {lower_quartile, upper_quartile};
}

template <typename T>
std::pair<T, T> RandomFuncs<T>::calculate_quartiles(std::vector<T> &data, bool lower_only)
{

    // Ensure the matrix is not empty
    if (data.empty())
        throw std::invalid_argument("The matrix is empty");

    // Calculate positions for the quartiles
    size_t n = data.size();
    T lower_quartile = 0, upper_quartile = 0;
    if (lower_only)
    {
        size_t lower_pos = static_cast<size_t>((n - 1) * 0.25);
        lower_quartile = RandomFuncs<T>::quick_select(data, lower_pos);
        return {lower_quartile, -1.0};
    }

    size_t upper_pos = static_cast<size_t>((n - 1) * 0.75);
    upper_quartile = RandomFuncs<T>::quick_select(data, upper_pos);

    return {lower_quartile, upper_quartile};
}

template <typename T>
std::pair<T, T> RandomFuncs<T>::calculate_quartiles(tbb::concurrent_vector<T> &data, bool lower_only)
{
    // Ensure the vector is not empty
    if (data.empty())
        throw std::invalid_argument("The vector is empty");

    tbb::parallel_sort(data.begin(), data.end());

    size_t n = data.size();
    T lower_quartile = 0, upper_quartile = 0;

    // Calculate the lower quartile
    size_t lower_pos = static_cast<size_t>((n - 1) * 0.25);
    lower_quartile = data[lower_pos];

    if (lower_only)
        return {lower_quartile, -1.0};

    // Calculate the upper quartile
    size_t upper_pos = static_cast<size_t>((n - 1) * 0.75);
    upper_quartile = data[upper_pos];

    return {lower_quartile, upper_quartile};
}

template <typename T>
std::pair<T, T> RandomFuncs<T>::outlier_check(tbb::concurrent_vector<T> &distances)
{
    // doing this to avoid the use of fringe elements
    if (distances.size() <= 4)
        return {std::numeric_limits<T>::lowest(), std::numeric_limits<T>::max()};

    T lower_q = 0.0, upper_q = 0.0;
    std::tie(lower_q, upper_q) = RandomFuncs<T>::calculate_quartiles(distances, false);

    return {lower_q, upper_q};
}

template struct RandomFuncs<double>;
template struct RandomFuncs<float>;

template <typename T>
AVector3TVec<T> TransformFuncs<T>::return_transformed(const SE3Type<T> &pose, const AVector3TVec<T> &points)
{
    if (points.empty())
    {
        if (params<T>.verbose)
            std::cout << "[INFO] TransformFuncs<T>::return_transformed the points vector is empty\n";
        return {};
    }

    AVector3TVec<T> out(points.size());
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, points.size()),
        [&](const tbb::blocked_range<std::size_t> &r)
        {
            for (std::size_t idx = r.begin(); idx < r.end(); ++idx)
                out[idx] = pose * points[idx];
        });

    return out;
}

template <typename T>
void TransformFuncs<T>::inplace(const SE3Type<T> &pose, const Point3dPtrVectCC<T> &points, T vox_size)
{
    if (points.empty())
    {
        if (params<T>.verbose)
            std::cout << "[INFO] TransformFuncs<T>::inplace the points vector is empty\n";
        return;
    }

    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, points.size()),
        [&](const tbb::blocked_range<std::size_t> &r)
        {
            for (std::size_t idx = r.begin(); idx < r.end(); ++idx)
            {
                points[idx]->point = pose * points[idx]->point;
                points[idx]->vox = Point3d<T>::calc_vox_index(points[idx]->point, vox_size);
                points[idx]->octant_key = Point3d<T>::sign_cardinality(points[idx]->point);
            }
        });
}

template <typename T>
void TransformFuncs<T>::moved_transform(const SE3Type<T> &pose, Point3dPtrVectCC<T> &points, Point3dPtrVect<T> &result, T vox_size)
{
    if (points.empty())
    {
        if (params<T>.verbose)
            std::cout << "[INFO] TransformFuncs<T>::inplace the points vector is empty\n";
        return;
    }

    result.clear();
    result.resize(points.size());

    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, points.size()),
        [&](const tbb::blocked_range<std::size_t> &r)
        {
            for (std::size_t idx = r.begin(); idx < r.end(); ++idx)
            {
                points[idx]->point = pose * points[idx]->point;
                points[idx]->vox = Point3d<T>::calc_vox_index(points[idx]->point, vox_size);
                points[idx]->octant_key = Point3d<T>::sign_cardinality(points[idx]->point);

                result[idx] = std::move(points[idx]);
            }
        });

    points.clear();
}

template struct TransformFuncs<double>;
template struct TransformFuncs<float>;

template <typename T>
Eigen::Matrix<T, 4, 1> PoseFuncs<T>::quat_to_vec(const Eigen::Quaternion<T> &v)
{
    Eigen::Matrix<T, 4, 1> ret(v.w(), v.x(), v.y(), v.z());
    ret.normalize();
    return ret;
}

template <typename T>
void PoseFuncs<T>::normalize_rotation_matrix(Eigen::Matrix<T, 3, 3> &rot)
{
    bool is_rotmat = false;
    {
        T determinant = rot.determinant();
        Eigen::Matrix<T, 3, 3> r_T_r = rot.transpose() * rot;
        is_rotmat = (std::abs(determinant - 1.0) < 1e-6) && (r_T_r.isApprox(Eigen::Matrix<T, 3, 3>::Identity(), 1e-6));
    }

    if (is_rotmat)
        return;

    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(rot, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<T, 3, 3> normalized_R = svd.matrixU() * svd.matrixV().transpose();
    rot = normalized_R;
}

template <typename T>
Eigen::Matrix<T, 3, 3> PoseFuncs<T>::skew_matrix(const Eigen::Matrix<T, 3, 1> &vector)
{
    Eigen::Matrix<T, 3, 3> matrix;
    (matrix << 0, -vector.z(), vector.y(),
     vector.z(), 0, -vector.x(),
     -vector.y(), vector.x(), 0)
        .finished();

    return matrix;
}

template <typename T>
typename PoseFuncs<T>::DynamicMatrix PoseFuncs<T>::remove_trace(const typename PoseFuncs<T>::DynamicMatrix &A)
{
    if (A.rows() != A.cols())
        throw std::invalid_argument("Matrix must be squared.");

    return (-A.trace() * DynamicMatrix::Identity(A.rows(), A.cols())) + A;
}

template <typename T>
typename PoseFuncs<T>::DynamicMatrix PoseFuncs<T>::remove_trace(const typename PoseFuncs<T>::DynamicMatrix &A, const typename PoseFuncs<T>::DynamicMatrix &B)
{
    if (A.rows() != B.rows() || A.cols() != B.cols())
        throw std::invalid_argument("Matrices A and B must have the same dimensions.");

    DynamicMatrix comb = B * A;
    return (PoseFuncs<T>::remove_trace(A) * PoseFuncs<T>::remove_trace(B)) + PoseFuncs<T>::remove_trace(comb);
}

template <typename T>
Eigen::Matrix<T, 4, 1> PoseFuncs<T>::rmat_to_quat(const Eigen::Matrix<T, 3, 3> &ori)
{
    Eigen::Quaternion<T> q(ori);
    Eigen::Matrix<T, 4, 1> q_v;
    // Set the order of coefficients to [w, x, y, z]
    q_v << q.w(), q.x(), q.y(), q.z();

    return q_v;
}

template <typename T>
Eigen::Quaternion<T> PoseFuncs<T>::vec_to_quat(const Eigen::Matrix<T, 4, 1> &v)
{
    return Eigen::Quaternion<T>(v(0), v(1), v(2), v(3));
}

template <typename T>
Eigen::Matrix<T, 3, 3> PoseFuncs<T>::quat_to_rmat(const Eigen::Matrix<T, 4, 1> &q_vec)
{
    Eigen::Quaternion<T> q = vec_to_quat(q_vec);
    return q.toRotationMatrix();
}

template <typename T>
Eigen::Matrix<T, 3, 1> PoseFuncs<T>::rmat_to_euler(const Eigen::Matrix<T, 3, 3> &R)
{
    Eigen::Matrix<T, 3, 1> euler_ang;

    T sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

    bool singular = sy < 1e-6; // If sy is close to zero, the matrix is singular

    if (!singular)
    {
        euler_ang(0) = std::atan2(R(2, 1), R(2, 2)); // roll (x-axis rotation)
        euler_ang(1) = std::atan2(-R(2, 0), sy);     // pitch (y-axis rotation)
        euler_ang(2) = std::atan2(R(1, 0), R(0, 0)); // yaw (z-axis rotation)
    }
    else
    {
        euler_ang(0) = std::atan2(-R(1, 2), R(1, 1)); // roll (x-axis rotation)
        euler_ang(1) = std::atan2(-R(2, 0), sy);      // pitch (y-axis rotation)
        euler_ang(2) = 0;                             // yaw (z-axis rotation)
    }

    return euler_ang;
}

template <typename T>
Eigen::Matrix<T, 3, 3> PoseFuncs<T>::exp_SO3(const Eigen::Matrix<T, 3, 1> &w, T dt)
{
    // Computes the vectorized exponential map for SO(3)
    Eigen::Matrix<T, 3, 1> phi = w * dt;
    Eigen::Matrix<T, 3, 3> A = PoseFuncs<T>::skew_matrix(phi);
    T theta = phi.norm();
    const T TOLERANCE = static_cast<T>(1e-10);

    if (theta < TOLERANCE)
        return Eigen::Matrix<T, 3, 3>::Identity();

    Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity() +
                               (std::sin(theta) / theta) * A +
                               ((1 - std::cos(theta)) / (theta * theta)) * A * A;
    return R;
}

template <typename T>
std::pair<Eigen::Matrix<T, 3, 3>, T> PoseFuncs<T>::omega_theta_frm_rot(const Eigen::Matrix<T, 3, 3> &rot)
{
    T acosinput = (rot.trace() - 1.0) / 2.0;
    T theta = 0.0;
    Eigen::Matrix<T, 3, 3> skew_sym;

    if (acosinput >= 1.0) // rotation is the identity matrix
        skew_sym.setZero();

    else if (acosinput <= -1.0)
    {
        theta = M_PI;
        // The rotation axis can be found from the diagonal elements of the matrix
        Eigen::Matrix<T, 3, 1> omega;
        if ((1.0 + rot(2, 2)) > 1e-5)
        {
            omega = (1.0 / std::sqrt(2.0 * (1.0 + rot(2, 2)))) * Eigen::Matrix<T, 3, 1>(rot(0, 2), rot(1, 2), 1.0 + rot(2, 2));
        }
        else if ((1.0 + rot(1, 1)) > 1e-5)
        {
            omega = (1.0 / std::sqrt(2.0 * (1.0 + rot(1, 1)))) * Eigen::Matrix<T, 3, 1>(rot(0, 1), 1.0 + rot(1, 1), rot(2, 1));
        }
        else
            omega = (1.0 / std::sqrt(2.0 * (1.0 + rot(0, 0)))) * Eigen::Matrix<T, 3, 1>(1.0 + rot(0, 0), rot(1, 0), rot(2, 0));

        skew_sym = PoseFuncs<T>::skew_matrix(omega);
    }
    else
    {
        theta = std::acos(acosinput);
        skew_sym = (rot - rot.transpose()) / (2.0 * std::sin(theta));
    }

    return {skew_sym, theta};
}

template <typename T>
Eigen::Matrix<T, 3, 3> PoseFuncs<T>::log_SO3(const Eigen::Matrix<T, 3, 3> &rot)
{
    auto rot_split = omega_theta_frm_rot(rot);
    return rot_split.first * rot_split.second;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PoseFuncs<T>::exp_SEK3(const Eigen::Matrix<T, Eigen::Dynamic, 1> &v)
{
    // ref: https://github.com/RossHartley/invariant-ekf/blob/master/src/LieGroup.cpp
    int K = (v.rows() - 3) / 3;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> X = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Identity(3 + K, 3 + K);
    Eigen::Matrix<T, 3, 3> R;
    Eigen::Matrix<T, 3, 3> Jl;
    Eigen::Matrix<T, 3, 1> w = v.head(3);
    T theta = w.norm();
    Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();

    if (theta < static_cast<T>(1e-5))
    {
        R = I;
        Jl = I;
    }
    else
    {
        Eigen::Matrix<T, 3, 3> A = PoseFuncs<T>::skew_matrix(w);
        T theta2 = theta * theta;
        T stheta = std::sin(theta);
        T ctheta = std::cos(theta);
        T oneMinusCosTheta2 = (1 - ctheta) / theta2;
        Eigen::Matrix<T, 3, 3> A2 = A * A;
        R = I + (stheta / theta) * A + oneMinusCosTheta2 * A2;
        Jl = I + oneMinusCosTheta2 * A + ((theta - stheta) / (theta2 * theta)) * A2;
    }

    X.block(0, 0, 3, 3) = R;
    for (int i = 0; i < K; ++i)
        X.block(0, 3 + i, 3, 1) = Jl * v.segment(3 + 3 * i, 3);

    return X;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PoseFuncs<T>::log_SEK3(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat)
{
    // identity rotation matrix
    Eigen::Matrix<T, 3, 3> omega;
    T theta;
    std::tie(omega, theta) = PoseFuncs<T>::omega_theta_frm_rot(mat.template block<3, 3>(0, 0));

    if (theta == 0.0)
    {
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> res = mat;
        res.template block<3, 3>(0, 0).setZero();
        for (size_t idx = 3; idx < size_t(mat.rows()); ++idx)
            res.row(idx).setZero();

        return res;
    }

    // Ginverse -> inverse left jacobian for current rotation
    Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();
    T cot_half_theta = 1.0 / std::tan(theta / 2.0);

    Eigen::Matrix<T, 3, 3> G_inv = (1.0 / theta) * I - 0.5 * omega + ((1.0 / theta) - 0.5 * (cot_half_theta)) * (omega * omega);

    // Prepare the result matrix and fill in the values
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> res = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(mat.rows(), mat.cols());
    res.template block<3, 3>(0, 0) = omega * theta;

    // fill up rest of matrix
    for (size_t idx = 3; idx < size_t(mat.cols()); ++idx)
        res.block(0, idx, 3, 1) = (G_inv * mat.block(0, idx, 3, 1)) * theta;

    return res;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PoseFuncs<T>::SEK3_inv(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &X)
{
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> inv = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Identity(X.rows(), X.cols());

    inv.template block<3, 3>(0, 0) = X.template block<3, 3>(0, 0).transpose();

    for (size_t idx = 3; idx < size_t(X.cols()); ++idx)
        inv.block(0, idx, 3, 1) = -inv.template block<3, 3>(0, 0) * X.block(0, idx, 3, 1);

    return inv;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PoseFuncs<T>::adj_SEK3(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &X)
{
    int K = X.cols() - 3;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Adj = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(3 + 3 * K, 3 + 3 * K);
    Eigen::Matrix<T, 3, 3> R = X.block(0, 0, 3, 3);

    Adj.block(0, 0, 3, 3) = R;
    for (int i = 0; i < K; ++i)
    {
        Adj.block(3 + 3 * i, 3 + 3 * i, 3, 3) = R;
        Adj.block(3 + 3 * i, 0, 3, 3) = PoseFuncs<T>::skew_matrix(X.block(0, 3 + i, 3, 1)) * R;
    }

    return Adj;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PoseFuncs<T>::adj_SEK3_LIE(const Eigen::Matrix<T, Eigen::Dynamic, 1> &X)
{
    size_t rows = X.rows();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> adj = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(rows, rows);

    adj.template block<3, 3>(0, 0) = PoseFuncs<T>::skew_matrix(X.segment(0, 3));
    for (size_t idx = 3; idx < rows; idx += 3)
    {
        adj.block(idx, idx, 3, 3) = adj.template block<3, 3>(0, 0);
        adj.block(idx, 0, 3, 3) = PoseFuncs<T>::skew_matrix(X.segment(idx, 3));
    }

    return adj;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PoseFuncs<T>::vec_to_SEK3(const Eigen::Matrix<T, 3, 3> &rmat, const Eigen::Matrix<T, Eigen::Dynamic, 1> &vecs)
{
    // Calculate the number of additional vectors
    const int K = vecs.rows() / 3;

    // Initialize the SEK3 matrix
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matt = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Identity(3 + K, 3 + K);

    // Set the top-left 3x3 block as the rotation matrix
    matt.template block<3, 3>(0, 0) = rmat;

    // Fill in the other vectors
    for (int idx = 0; idx < vecs.rows(); idx += 3)
        matt.template block<3, 1>(0, 3 + (idx / 3)) = vecs.segment(idx, 3);

    return matt;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> PoseFuncs<T>::SEK3_to_vec(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matt, bool from_diff)
{
    size_t cols_post_rot = matt.cols() - 3;

    size_t K = 3 + (cols_post_rot * 3);
    Eigen::Matrix<T, Eigen::Dynamic, 1> vec = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(K, 1);
    Eigen::Matrix<T, 3, 3> rot = matt.template block<3, 3>(0, 0);
    if (!rot.isZero())
    {
        if (from_diff)
            vec.segment(0, 3) << rot(2, 1), rot(0, 2), rot(1, 0);
        else
            vec.segment(0, 3) = Sophus::SO3<T>(matt.template block<3, 3>(0, 0)).log();
    }

    size_t vec_loc = 3;
    for (size_t idx = 3; idx < size_t(matt.cols()); ++idx)
    {
        vec.segment(vec_loc, 3) = matt.block(0, idx, 3, 1);
        vec_loc += 3;
    }

    return vec;
}

template <typename T>
Eigen::Matrix<T, 3, 1> PoseFuncs<T>::rmat_to_vec(const Eigen::Matrix<T, 3, 3> &rmat)
{
    // Ensure the matrix is a proper rotation matrix
    Eigen::AngleAxis<T> aa(rmat);
    return aa.angle() * aa.axis();
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PoseFuncs<T>::left_jacobian_inverse(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &adx, size_t num_terms)
{
    using DynamicEig = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
    DynamicEig res = DynamicEig::Zero(adx.rows(), adx.cols());
    DynamicEig ad_pow = DynamicEig::Identity(adx.rows(), adx.cols());

    // intialize factorial and sign
    T factorial = 1.0;
    for (size_t i = 0; i < num_terms; ++i)
    {
        if (i > 0)
            factorial *= static_cast<T>(i + 1.0);

        // Calculate the current term and add it to the result matrix
        T coefficient = std::pow(-1.0, i) / factorial;
        res.noalias() += coefficient * ad_pow;

        // Update ad_pow by multiplying with adx
        ad_pow = ad_pow * adx;
    }

    return res;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> PoseFuncs<T>::exp_matrix(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat, size_t num_terms)
{
    using DynamicAMat = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
    DynamicAMat res = DynamicAMat::Identity(mat.rows(), mat.cols());
    DynamicAMat mat_power = res;

    T fac = 1.0;
    for (size_t idx = 1; idx < num_terms; ++idx)
    {
        mat_power = mat_power * mat;
        fac *= T(idx);
        res.noalias() += mat_power / fac;
    }

    return res;
}

template <typename T>
std::shared_ptr<std::array<Eigen::Matrix<T, 3, 3>, 3>> PoseFuncs<T>::rot_der(const Eigen::Matrix<T, 3, 3> &w)
{
    // convert to quaternion
    Eigen::Matrix<T, 4, 1> q = PoseFuncs<T>::rmat_to_quat(w);

    auto dR = std::make_shared<std::array<Eigen::Matrix<T, 3, 3>, 3>>();

    ((*dR)[0] << 2 * q(1), 2 * q(2), 2 * q(3),
     2 * q(2), -2 * q(1), -2 * q(0),
     2 * q(3), 2 * q(0), -2 * q(1))
        .finished();

    ((*dR)[1] << -2 * q(2), 2 * q(1), 2 * q(0),
     2 * q(1), 2 * q(2), 2 * q(3),
     -2 * q(0), 2 * q(3), -2 * q(2))
        .finished();

    ((*dR)[2] << -2 * q(3), -2 * q(0), 2 * q(1),
     2 * q(0), -2 * q(3), 2 * q(2),
     2 * q(1), 2 * q(2), 2 * q(3))
        .finished();

    return dR;
}

template <typename T>
void PoseFuncs<T>::tester()
{
    Eigen::Matrix<T, 3, 1> base_rotation(0.5, 0.5, 0.5);
    Eigen::Matrix<T, 3, 1> position(-0.5, 0.25, 0.1);
    Eigen::Matrix<T, 3, 1> velocity(0.25, 0.36, 0.9);

    Eigen::Matrix<T, 3, 3> rmat = PoseFuncs<T>::exp_SO3(base_rotation, 1.0);
    std::cout << "Rotation matrix:\n"
              << rmat << std::endl;
    Eigen::Matrix<T, 9, 1> val;
    (val << base_rotation, position, velocity).finished();

    // convert to se3
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> se_3 = PoseFuncs<T>::exp_SEK3(val);
    std::cout << "\nSE_3 value:\n"
              << se_3 << std::endl;

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> log_se_3 = PoseFuncs<T>::log_SEK3(se_3);
    std::cout << "\nLogSE_3 value:\n"
              << log_se_3 << std::endl;

    // alternative approach
    se_3 = PoseFuncs<T>::vec_to_SEK3(rmat, val.segment(3, 6));
    std::cout << "\nSE_3 value alternative method:\n"
              << se_3 << std::endl;

    // convert se3 to vec
    Eigen::Matrix<T, Eigen::Dynamic, 1> res = PoseFuncs<T>::SEK3_to_vec(se_3);
    std::cout << "\nSE_3 to vector format: " << res.transpose() << std::endl;
    std::cout << "It should be similar to: " << res.transpose() << std::endl;

    // calculating adjoint
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> adj = PoseFuncs<T>::adj_SEK3(se_3);
    std::cout << "\nAdjoint value:\n"
              << adj << std::endl;

    // calculating adjoint
    std::cout << "Vec of log_se_3: \n"
              << PoseFuncs<T>::SEK3_to_vec(log_se_3, true) << std::endl;

    adj = PoseFuncs<T>::adj_SEK3_LIE(PoseFuncs<T>::SEK3_to_vec(log_se_3, true));
    std::cout << "\nAdjoint value for lie:\n"
              << adj << std::endl;
}

template struct PoseFuncs<double>;
template struct PoseFuncs<float>;

template <typename T>
std::string PrintFuncs<T>::Tvec_to_string(const Eigen::Matrix<T, Eigen::Dynamic, 1> &vec)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    for (int i = 0; i < vec.size(); ++i)
        oss << std::setw(8) << vec(i) << " ";

    return oss.str();
}

template struct PrintFuncs<double>;
template struct PrintFuncs<float>;

template <typename T>
EMAAveraging<T>::EMAAveraging(size_t window_size)
{
    window_num = window_size;
    weights.resize(window_size);

    T alpha = 2.0 / (window_size + 1);
    for (size_t idx = 0; idx < window_size; ++idx)
        weights[idx] = alpha * std::pow(1 - alpha, idx);

    // reversing the weights so most recent get highest weight
    std::reverse(weights.begin(), weights.end());
}

template <typename T>
typename PoseFuncs<T>::DynamicMatrix EMAAveraging<T>::updated_matrix(const typename PoseFuncs<T>::DynamicMatrix &c_mat)
{
    // populate vector
    window.emplace_back(c_mat);

    if (window.size() == 1)
        return c_mat;

    if (window.size() > window_num)
        window.pop_front();

    // Compiling windowed average
    typename PoseFuncs<T>::DynamicMatrix res = PoseFuncs<T>::DynamicMatrix::Zero(c_mat.rows(), c_mat.cols());
    size_t m_mat = window.size() - 1;
    T tot_weight = 0.0;

    for (size_t w_idx = window_num - 1; w_idx > (window_num - window.size()); --w_idx)
    {
        tot_weight += weights[w_idx];
        res.noalias() += weights[w_idx] * window[m_mat];
        --m_mat;
    }

    // just normalizing the weights
    return res / tot_weight;
}

template struct EMAAveraging<double>;
template struct EMAAveraging<float>;

template <typename T>
ScalarAveraging<T>::ScalarAveraging(size_t window_size)
{
    window_num = window_size;
    weights.resize(window_size);

    T alpha = 2.0 / (window_size + 1);
    for (size_t idx = 0; idx < window_size; ++idx)
        weights[idx] = alpha * std::pow(1 - alpha, idx);

    // reversging the weights so most recent get highest weight
    std::reverse(weights.begin(), weights.end());
}

template <typename T>
T ScalarAveraging<T>::updated_matrix(T value)
{
    // populate vector
    window.emplace_back(value);

    if (window.size() == 1)
        return value;

    if (window.size() > window_num)
        window.pop_front();

    // Compiling windowed average
    T res = 0;
    size_t m_mat = window.size() - 1;
    T tot_weight = 0.0;

    for (size_t w_idx = window_num - 1; w_idx > (window_num - window.size()); --w_idx)
    {
        tot_weight += weights[w_idx];
        res += weights[w_idx] * window[m_mat];
        --m_mat;
    }

    // just normalizing the weights
    return res / tot_weight;
}

template struct ScalarAveraging<double>;
template struct ScalarAveraging<float>;