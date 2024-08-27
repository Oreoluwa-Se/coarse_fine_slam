#ifndef CALC_HELPERS_HPP
#define CALC_HELPERS_HPP

#include <atomic>
#include <map_storage/utils/alias.hpp>
#include <memory>
#include <tbb/concurrent_vector.h>
#include <tbb/mutex.h>
#include <deque>

template <typename T>
struct RandomFuncs
{
    static T quick_select(std::vector<T> &data, size_t k);

    static std::pair<T, T> calculate_quartiles(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matrix, bool lower_only = true);

    static std::pair<T, T> calculate_quartiles(std::vector<T> &matrix, bool lower_only = true);

    static std::pair<T, T> calculate_quartiles(tbb::concurrent_vector<T> &matrix, bool lower_only = true);

    static std::pair<T, T> outlier_check(tbb::concurrent_vector<T> &distances);
};

template <typename T>
struct TransformFuncs
{
    static AVector3TVec<T> return_transformed(const SE3Type<T> &pose, const AVector3TVec<T> &points);

    static void inplace(const SE3Type<T> &pose, const Point3dPtrVectCC<T> &points, T vox_size = 1.0);

    static void moved_transform(const SE3Type<T> &pose, Point3dPtrVectCC<T> &points, Point3dPtrVect<T> &result, T vox_size = 1.0);
};

template <typename T>
struct PoseFuncs
{
    using DynamicMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
    using DerPtr = std::shared_ptr<std::array<Eigen::Matrix<T, 3, 3>, 3>>;

    static void normalize_rotation_matrix(Eigen::Matrix<T, 3, 3> &rot);

    static Eigen::Matrix<T, 3, 3> skew_matrix(const Eigen::Matrix<T, 3, 1> &vector);

    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> remove_trace(const DynamicMatrix &A);

    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> remove_trace(const DynamicMatrix &A, const DynamicMatrix &B);

    static Eigen::Matrix<T, 4, 1> rmat_to_quat(const Eigen::Matrix<T, 3, 3> &ori);

    static Eigen::Quaternion<T> vec_to_quat(const Eigen::Matrix<T, 4, 1> &v);

    static Eigen::Matrix<T, 3, 3> quat_to_rmat(const Eigen::Matrix<T, 4, 1> &q_vec);

    static Eigen::Matrix<T, 3, 1> rmat_to_euler(const Eigen::Matrix<T, 3, 3> &R);

    static Eigen::Matrix<T, 4, 1> quat_to_vec(const Eigen::Quaternion<T> &v);

    static Eigen::Matrix<T, 3, 3> exp_SO3(const Eigen::Matrix<T, 3, 1> &w, T dt);

    static Eigen::Matrix<T, 3, 3> log_SO3(const Eigen::Matrix<T, 3, 3> &rot);

    static Eigen::Matrix<T, 3, 1> rmat_to_vec(const Eigen::Matrix<T, 3, 3> &rmat);

    static std::shared_ptr<std::array<Eigen::Matrix<T, 3, 3>, 3>> rot_der(const Eigen::Matrix<T, 3, 3> &w);

    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> exp_SEK3(const Eigen::Matrix<T, Eigen::Dynamic, 1> &v);

    static std::pair<Eigen::Matrix<T, 3, 3>, T> omega_theta_frm_rot(const Eigen::Matrix<T, 3, 3> &rot);

    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> log_SEK3(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat);

    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> vec_to_SEK3(const Eigen::Matrix<T, 3, 3> &rmat, const Eigen::Matrix<T, Eigen::Dynamic, 1> &vecs);

    static Eigen::Matrix<T, Eigen::Dynamic, 1> SEK3_to_vec(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matt, bool frm_diff = false);

    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> adj_SEK3(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &X);

    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> adj_SEK3_LIE(const Eigen::Matrix<T, Eigen::Dynamic, 1> &X);

    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> SEK3_inv(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &X);

    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> left_jacobian_inverse(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &adx, size_t num_terms);

    static Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> exp_matrix(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat, size_t num_terms);

    static void tester();
};

template <typename T>
struct PrintFuncs
{
    static std::string Tvec_to_string(const Eigen::Matrix<T, Eigen::Dynamic, 1> &vec);
};

template <typename T>
struct EMAAveraging
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<EMAAveraging<T>>;

    EMAAveraging(size_t window_size);

    typename PoseFuncs<T>::DynamicMatrix updated_matrix(const typename PoseFuncs<T>::DynamicMatrix &c_mat);

private:
    std::deque<typename PoseFuncs<T>::DynamicMatrix> window;
    std::vector<T> weights;
    size_t window_num;
};

template <typename T>
using EMAPtr = typename EMAAveraging<T>::Ptr;

template <typename T>
struct ScalarAveraging
{
    using Ptr = std::unique_ptr<ScalarAveraging<T>>;

    ScalarAveraging(size_t window_size);

    T updated_matrix(T value);

private:
    std::deque<T> window;
    std::vector<T> weights;
    size_t window_num;
};

template <typename T>
using SAPtr = typename ScalarAveraging<T>::Ptr;
#endif