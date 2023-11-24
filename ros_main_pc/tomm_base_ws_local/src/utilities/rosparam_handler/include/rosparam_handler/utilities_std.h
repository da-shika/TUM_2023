#ifndef ROSPARAM_HANDLER_UTILITIES_STD_H
#define ROSPARAM_HANDLER_UTILITIES_STD_H

#include <limits>
#include <stdlib.h>
#include <rosparam_handler/utilities.h>

/// \brief Helper function to test for std::vector
template <typename T>
using is_vector = std::is_same<T, std::vector<typename T::value_type, typename T::allocator_type>>;

/// \brief Helper function to test for std::map
template <typename T>
using is_map = std::is_same<
    T, std::map<typename T::key_type, typename T::mapped_type, typename T::key_compare, typename T::allocator_type>>;

namespace utilities_std {

template <typename T>
inline std::string to_string(const std::vector<T>& v) {
    std::string s;
    if (!v.empty()) {
        std::stringstream ss;
        ss << '[';
        std::copy(v.begin(), v.end(), std::ostream_iterator<T>(ss, ", "));
        s = ss.str();
        s.erase(s.size() - 2);
        s += ']';
    }
    return s;
}

template <typename... T>
inline std::string to_string(const std::map<T...>& map) {
    std::stringstream ss;
    ss << '{';
    for (typename std::map<T...>::const_iterator it = map.begin(); it != map.end(); ++it) {
        ss << (*it).first << " --> " << (*it).second << ", ";
    }
    ss << '}';
    return ss.str();
}

template <typename T>
inline bool from_string(const std::string& s, std::vector<T>& val) {
  if(s.empty())
    return false;

  std::string s_cleaned = s;
  if(s_cleaned.front() == '[')
    s_cleaned.erase(0, 1);
  if(s_cleaned.back() == ']')
    s_cleaned.erase(s_cleaned.size() - 1);

  std::vector<T> res;
  std::string token;
  std::istringstream token_stream(s_cleaned);

  while (std::getline(token_stream, token, ','))
  {
    try {
      res.push_back(std::stod(token));
    } catch (const std::invalid_argument& ia) {
      ROS_ERROR_STREAM("from_string(): Invalid argument:" << ia.what());
      return false;
    }
  }
  
  val = res;
  return true;
}

/// \brief Set parameter on ROS parameter server
///
/// \param key Parameter name
/// \param val Parameter value
template <typename T>
inline void setParam(const std::string key, T val) {
    ros::param::set(key, val);
}

/// \brief Get parameter from ROS parameter server
///
/// \param key Parameter name
/// \param val Parameter value
template <typename T>
inline bool getParam(const std::string key, T& val) {
    if (!ros::param::has(key)) {
        ROS_ERROR_STREAM("Parameter '" << key << "' is not defined.");
        return false;
    } else if (!ros::param::get(key, val)) {
        ROS_ERROR_STREAM("Could not retrieve parameter'" << key << "'. Does it have a different type?");
        return false;
    } else {
        return true;
    }
}

/// \brief Get parameter from ROS parameter server or use default value
///
/// If parameter does not exist on server yet, the default value is used and set on server.
/// \param key Parameter name
/// \param val Parameter value
/// \param defaultValue Parameter default value
template <typename T>
inline bool getParam(const std::string key, T& val, const T& defaultValue) {
    if (!ros::param::has(key) || !ros::param::get(key, val)) {
        val = defaultValue;
        ros::param::set(key, defaultValue);
        ROS_INFO_STREAM("Setting default value for parameter '" << key << "'.");
        return true;
    } else {
        // Param was already retrieved with last if statement.
        return true;
    }
}

/// \brief Limit parameter to lower bound if parameter is a scalar.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
inline void testMin(const std::string key, T& val, T min = std::numeric_limits<T>::min()) {
    if (val < min) {
        ROS_WARN_STREAM("Value of " << val << " for " << key
                                    << " is smaller than minimal allowed value. Correcting value to min=" << min);
        val = min;
    }
}

/// \brief Limit parameter to lower bound if parameter is a vector.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
inline void testMin(const std::string key, std::vector<T>& val, T min = std::numeric_limits<T>::min()) {
    for (auto& v : val)
        testMin(key, v, min);
}

/// \brief Limit parameter to lower bound if parameter is a map.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename K, typename T>
inline void testMin(const std::string key, std::map<K, T>& val, T min = std::numeric_limits<T>::min()) {
    for (auto& v : val)
        testMin(key, v.second, min);
}

/// \brief Limit parameter to upper bound if parameter is a scalar.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
inline void testMax(const std::string key, T& val, T max = std::numeric_limits<T>::max()) {
    if (val > max) {
        ROS_WARN_STREAM("Value of " << val << " for " << key
                                    << " is greater than maximal allowed. Correcting value to max=" << max);
        val = max;
    }
}

/// \brief Limit parameter to upper bound if parameter is a vector.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename T>
inline void testMax(const std::string key, std::vector<T>& val, T max = std::numeric_limits<T>::max()) {
    for (auto& v : val)
        testMax(key, v, max);
}

/// \brief Limit parameter to upper bound if parameter is a map.
///
/// \param key Parameter name
/// \param val Parameter value
/// \param min Lower Threshold
template <typename K, typename T>
inline void testMax(const std::string key, std::map<K, T>& val, T max = std::numeric_limits<T>::max()) {
    for (auto& v : val)
        testMax(key, v.second, max);
}

} // namespace utilities_std

#endif
