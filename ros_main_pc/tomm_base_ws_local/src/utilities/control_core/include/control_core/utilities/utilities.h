/*! \file
 *
 * \author Simon Armleder
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#ifndef CONTROL_CORE_UTILITIES_H
#define CONTROL_CORE_UTILITIES_H

#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
	
namespace cc
{

  //////////////////////////////////////////////////////////////////////////////
  // std vectors
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief check if vector contains key 
   * 
   * @tparam T 
   * @param vector 
   * @param key 
   * @return true 
   * @return false 
   */
  template<typename T>
  inline bool has(const std::vector<T>& vector, const T& key)
  {
    if(std::find(vector.cbegin(), vector.cend(), key) != vector.cend())
    {
      return true;
    }
    return false;
  }
  
  /**
   * @brief fill linearly spaced vector
   * 
   * @tparam T 
   * @param low 
   * @param high 
   * @param n 
   * @return std::vector<T> 
   */
  template<typename T>
  inline std::vector<T> linspace(T low, T high, std::size_t n) {
    std::vector<double> res(n);
    T step = (high - low) / static_cast<T>(n - 1);
    for (auto it = res.begin(); it != res.end(); ++it) 
    {
      *it = low;
      low += step;
    }
    return res;
  }

  template<typename T>
  inline void remove(std::vector<T>& vector, const T& key)
  {
    vector.erase(std::remove(vector.begin(), vector.end(), key), vector.end());
  }

  template<typename T>
  inline int findIndex(std::vector<T>& vector, const T& key)
  {
    return std::distance(vector.begin(), std::find(vector.begin(), vector.end(), key));
  }

  /**
   * @brief sorts vector in accending order and returns indices
   * 
   * @tparam T 
   * @param v 
   * @return std::vector<size_t> 
   */
  template <typename T>
  inline std::vector<size_t> sort_indexes(const std::vector<T> &v) 
  {
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::stable_sort(idx.begin(), idx.end(),
        [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
    return idx;
  }

  /**
   * @brief insert v into v_into
   * 
   * Defaults append an resize v_b.
   * If idx given, insert at index. Note in this case v_into needs to have enough size.
   */
  template<typename T>
  inline void insert(const std::vector<T>& v, std::vector<T>& v_into, int idx=-1)
  {
    if(idx < 0)
      std::copy(v.begin(), v.end(), std::back_inserter(v_into));
    else
      v_into.insert(v_into.begin() + idx, v.begin(), v.end());
  }

  //////////////////////////////////////////////////////////////////////////////
  // maps
  //////////////////////////////////////////////////////////////////////////////

  template<typename Key, typename T>
  inline bool has(const std::unordered_map<Key, T>& map, const Key& key)
  {
    return map.find(key) != map.end();
  }

  //////////////////////////////////////////////////////////////////////////////
  // strings
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief convert std::vector to string
   * 
   * @tparam T 
   * @param vector 
   * @return std::string 
   */
  template<typename T>
  inline std::string toString(const std::vector<T>& vector)
  {
    std::stringstream ss;

    for(size_t i = 0; i < vector.size()-1; ++i)
      ss << vector[i] << ", ";
    ss << vector.back();
    return ss.str(); 
  }

  /**
   * @brief split string into vector of tokens based on delimiter
   * 
   * @param s 
   * @param delimiter 
   * @return std::vector<std::string> 
   */
  inline std::vector<std::string> split(const std::string& s, char delimiter)
  {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream token_stream(s);
    while (std::getline(token_stream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
  }

  /**
   * @brief check if string 1 contains string 2 as substring
   * 
   * @param s1 
   * @param s2 
   * @return true 
   * @return false 
   */
  inline bool contains(const std::string& s1, const std::string& s2)
  {
    return s1.find(s2) != std::string::npos;
  }

}

#endif
