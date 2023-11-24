#ifndef ROSPARAM_HANDLER_UTILITIES_EIGEN_H
#define ROSPARAM_HANDLER_UTILITIES_EIGEN_H

#include <Eigen/Dense>
#include <rosparam_handler/utilities_std.h>

namespace utilities_eigen {

template <typename _Derived>
inline std::string to_string(const Eigen::MatrixBase<_Derived>& v) {
    std::stringstream ss;
    ss << '[';
    for (int i = 0; i < v.size(); ++i) {
        ss << v[i] << ", ";
    }
    std::string s = ss.str();
    s.erase(s.size() - 2);
    s += ']';
    return s;
}

template <typename _Derived>
inline std::string to_string(const Eigen::QuaternionBase<_Derived>& v) {
    std::stringstream ss;
    ss << '[';
    for (int i = 0; i < 4; ++i) {
        ss << v.derived().coeffs()[i] << ", ";
    }
    std::string s = ss.str();
    s.erase(s.size() - 2);
    s += ']';
    return s;
}

template <typename _Derived>
inline bool from_vector(const std::vector<typename _Derived::Scalar>& v, Eigen::MatrixBase<_Derived>& val) {
    typedef typename _Derived::Index Index;
    typedef typename _Derived::Scalar Scalar;

    Index Rows, Cols, Size;
    Rows = _Derived::RowsAtCompileTime;
    Cols = _Derived::ColsAtCompileTime;
    Size = _Derived::SizeAtCompileTime;

    if (Rows == Eigen::Dynamic || Cols == Eigen::Dynamic) {
        if (Rows == Eigen::Dynamic && Cols != Eigen::Dynamic) {
            // rows are free
            val.derived().resize(v.size(), Cols);
        } else if (Cols == Eigen::Dynamic && Rows != Eigen::Dynamic) {
            // cols are free
            val.derived().resize(Rows, v.size());
        } else {
            // default case, both are free, make nx1 dim vector
            val.derived().resize(v.size(), 1);
        }
    } else {
        // matrix is fixed size, check dimension matches
        if (Index(v.size()) != Size) {
            ROS_ERROR("Vec has dim=%ld != desired=%ld", v.size(), Size);
            return false;
        }
    }

    // store the data
    val.setZero();
    for (Index i = 0; i < val.size(); ++i) {
        val(i) = v[i];
    }
    return true;
}

template <typename _Derived>
inline bool from_vector(const std::vector<typename _Derived::Scalar>& v, Eigen::QuaternionBase<_Derived>& val) {
    // check the size
    if (v.size() != std::size_t(4)) {
        ROS_ERROR("from_vector: could not load Eigen::Matrix, because loaded has dim=%ld != desired=%ld", v.size(),
                  std::size_t(4));
        return false;
    }

    // store the data
    for (size_t i = 0; i < v.size(); ++i) {
        val.derived().coeffs()[i] = v[i];
    }
    return true;
}

template <typename _Derived>
inline bool from_string(const std::string& str, Eigen::MatrixBase<_Derived>& val) {
    typedef typename _Derived::Index Index;

    std::vector<typename _Derived::Scalar> vec;
    if (!utilities_std::from_string(str, vec)) {
        ROS_ERROR("from_string(): Failed to convert '%s' to vector.", str.c_str());
    }

    if (Index(vec.size()) != val.size()) {
        ROS_ERROR("from_string(): Vec '%s' has dim=%ld != desired=%ld", str.c_str(), vec.size(), val.size());
        return false;
    }
    return from_vector(vec, val);
}

template <typename _Derived>
inline bool from_string(const std::string& str, Eigen::QuaternionBase<_Derived>& val) {
    typedef typename _Derived::Coefficients::Index Index;

    std::vector<typename _Derived::Scalar> vec;
    if (!utilities_std::from_string(str, vec)) {
        ROS_ERROR("from_string(): Failed to convert '%s' to vector.", str.c_str());
    }

    if (Index(vec.size()) != 4) {
        ROS_ERROR("from_string(): Vec '%s' has dim=%ld != desired=%d", str.c_str(), vec.size(), 4);
        return false;
    }
    return from_vector(vec, val);
}

/**
 * @brief Set matrix on ROS parameter server
 *
 * @tparam _Derived
 * @param key
 * @param val
 */
template <typename _Derived>
inline void setParam(const std::string& key, Eigen::MatrixBase<_Derived>& val) {
    typedef typename _Derived::Scalar Scalar;
    utilities_std::setParam(key, std::vector<Scalar>(val.derived().data(), val.derived().data() + val.size()));
}

template <typename _Derived>
inline void setParam(const std::string& key, Eigen::QuaternionBase<_Derived>& val) {
    typedef typename _Derived::Scalar Scalar;
    utilities_std::setParam(key, std::vector<Scalar>(val.derived().coeffs().data(),
                                                     val.derived().coeffs().data() + val.derived().coeffs().size()));
}

/**
 * @brief get eigen matrix from parameter server
 *
 * @tparam _Derived
 * @param key
 * @param val
 * @return true
 * @return false
 */
template <typename _Derived>
inline bool getParam(const std::string& key, Eigen::MatrixBase<_Derived>& val) {
    typedef typename _Derived::Scalar Scalar;
    std::vector<Scalar> v;
    if (!utilities_std::getParam(key, v)) {
        ROS_ERROR("load: could not load Eigen::Matrix '%s'.", key.c_str());
        return false;
    }
    return from_vector(v, val);
}

/**
 * @brief get eigen matrix from parameter server
 *
 * @tparam _Derived
 * @param key
 * @param val
 * @param defaultValue
 * @return true
 * @return false
 */
template <typename _Derived>
inline bool getParam(const std::string& key, Eigen::MatrixBase<_Derived>& val,
                     const std::vector<typename _Derived::Scalar>& defaultValue) {
    typedef typename _Derived::Scalar Scalar;

    if (!ros::param::has(key) || !getParam(key, val)) {
        val.derived().resize(defaultValue.size());
        for (int i = 0; i < defaultValue.size(); ++i)
            val.derived()[i] = defaultValue[i];
        setParam(key, val);
        ROS_INFO_STREAM("Setting default value for parameter '" << key << "'.");
        return true;
    } else {
        // Param was already retrieved with last if statement.
        return true;
    }
}

/**
 * @brief Get quaternion from parameter server
 *
 * @tparam _Derived
 * @param key
 * @param val
 * @return true
 * @return false
 */
template <typename _Derived>
inline bool getParam(const std::string& key, Eigen::QuaternionBase<_Derived>& val) {
    typedef typename _Derived::Scalar Scalar;
    std::vector<Scalar> v;
    if (!utilities_std::getParam(key, v)) {
        ROS_ERROR("load: could not load Eigen::QuaternionBase '%s'.", key.c_str());
        return false;
    }
    return from_vector(v, val);
}

/**
 * @brief Get quaternion from parameter server
 *
 * @tparam _Derived
 * @param key
 * @param val
 * @param defaultValue
 * @return true
 * @return false
 */
template <typename _Derived>
inline bool getParam(const std::string& key, Eigen::QuaternionBase<_Derived>& val,
                     const std::vector<typename _Derived::Scalar>& defaultValue) {
    typedef typename _Derived::Scalar Scalar;

    if (!ros::param::has(key) || !getParam(key, val)) {
        if (defaultValue.size() != 4) {
            ROS_INFO_STREAM("Setting default value for parameter '" << key << "' failed, dimension not 4");
            return false;
        }
        for (int i = 0; i < 4; ++i)
            val.derived().coeffs()[i] = defaultValue[i];
        setParam(key, val);
        ROS_INFO_STREAM("Setting default value for parameter '" << key << "'.");
        return true;
    } else {
        // Param was already retrieved with last if statement.
        return true;
    }
}

/**
 * @brief Limit parameter to lower bound if parameter is a Matrix
 *
 * @tparam _Derived
 * @param key
 * @param val
 * @param min
 */
template <typename _Derived>
inline void testMin(const std::string key, Eigen::MatrixBase<_Derived>& val,
                    typename Eigen::MatrixBase<_Derived>::Scalar min =
                        std::numeric_limits<typename Eigen::MatrixBase<_Derived>::Scalar>::min()) {
    for (int i = 0; i < val.size(); ++i)
        utilities_std::testMin(key, val[i], min);
}

/**
 * @brief Limit parameter to lower bound if parameter is a Quaternion
 *
 * @tparam _Derived
 * @param key
 * @param val
 * @param min
 */
template <typename _Derived>
inline void testMin(const std::string key, Eigen::QuaternionBase<_Derived>& val,
                    typename Eigen::QuaternionBase<_Derived>::Scalar min =
                        std::numeric_limits<typename Eigen::QuaternionBase<_Derived>::Scalar>::min()) {
    for (int i = 0; i < 4; ++i)
        utilities_std::testMin(key, val.derived().coeffs()[i], min);
}

/**
 * @brief Limit parameter to lower bound if parameter is a Matrix
 *
 * @tparam _Derived
 * @param key
 * @param val
 * @param max
 */
template <typename _Derived>
inline void testMax(const std::string key, Eigen::MatrixBase<_Derived>& val,
                    typename Eigen::MatrixBase<_Derived>::Scalar max =
                        std::numeric_limits<typename Eigen::MatrixBase<_Derived>::Scalar>::max()) {
    for (int i = 0; i < val.size(); ++i)
        utilities_std::testMax(key, val[i], max);
}

/**
 * @brief Limit parameter to lower bound if parameter is a Quaternion
 *
 * @tparam _Derived
 * @param key
 * @param val
 * @param max
 */
template <typename _Derived>
inline void testMax(const std::string key, Eigen::QuaternionBase<_Derived>& val,
                    typename Eigen::QuaternionBase<_Derived>::Scalar max =
                        std::numeric_limits<typename Eigen::QuaternionBase<_Derived>::Scalar>::max()) {
    for (int i = 0; i < 4; ++i)
        utilities_std::testMax(key, val.derived().coeffs()[i], max);
}

} // namespace utilities_eigen

#endif
