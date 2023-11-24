#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <type_traits>

// for color print
#define P_RED "\033[31m"    /* Red */
#define P_YELLOW "\033[33m" /* Yellow */
#define P_WHITE "\033[0m"   /* White */

#define PRINT(str) std::cout << P_WHITE << str << std::endl
#define PRINT_ERROR(str) std::cout << P_RED << "ERROR: " << str << P_WHITE << std::endl
#define PRINT_WARN(str) std::cout << P_YELLOW << "WARN: " << str << P_WHITE << std::endl

/**
 * @brief Conversion Functions for additional types: Eigen matrices, quaternions, SE3, etc.
 *
 */
namespace YAML
{
  /**
   * @brief Decode function for Eigen::Matrix type
   *
   * @tparam _Scalar
   * @tparam _Rows
   * @tparam _Cols
   * @tparam _Options
   * @tparam _MaxRows
   * @tparam _MaxCols
   */
  template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
  struct convert<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>>
  {
  public:
    static bool decode(const Node &node, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &rhs)
    {
      if (!node.IsSequence())
        return false;

      if (_Rows == Eigen::Dynamic || _Cols == Eigen::Dynamic)
      {
        if (_Rows == Eigen::Dynamic && _Cols != Eigen::Dynamic)
        {
          // rows are free
          rhs.resize(node.size(), _Cols);
        }
        else if (_Cols == Eigen::Dynamic && _Rows != Eigen::Dynamic)
        {
          // cols are free
          rhs.resize(_Rows, node.size());
        }
        else
        {
          // default case, both are free, make nx1 dim vector
          rhs.resize(node.size(), 1);
        }
      }
      else
      {
        // matrix is fixed size, check dimension
        if (node.size() != _Rows * _Cols)
        {
          PRINT_ERROR("load: could not load Eigen::Matrix, because loaded has dim=" << node.size() << " != desired=" << _Rows * _Cols);
          return false;
        }
      }

      // store data
      rhs.setZero();
      for (size_t i = 0; i < rhs.size(); ++i)
      {
        rhs(i) = node[i].as<double>();
      }
      return true;
    }
  };

  /**
   * @brief Decode function for Quaternion type
   *
   * @tparam _Scalar
   * @tparam _Options
   */
  template <typename _Scalar, int _Options>
  struct convert<Eigen::Quaternion<_Scalar, _Options>>
  {
  public:
    static bool decode(const Node &node, Eigen::Quaternion<_Scalar, _Options> &rhs)
    {
      if (!node.IsSequence() || node.size() != 4)
      {
        PRINT_ERROR("Quaternion has wrong size: size = " << node.size());
        return false;
      }

      // store data
      rhs.x() = node[0].as<double>();
      rhs.y() = node[1].as<double>();
      rhs.z() = node[2].as<double>();
      rhs.w() = node[3].as<double>();

      return true;
    }
  };
}

/**
 * @brief Utility functions for yaml parameter loading
 *
 */
namespace yaml
{
  /**
   * @brief split string into vector of tokens based on delimiter
   *
   * @param s
   * @param delimiter
   * @return std::vector<std::string>
   */
  inline std::vector<std::string> split(const std::string &s, char delimiter)
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
   * @brief Retrieve parameter from config without type conversion
   *
   * @param config
   * @param name
   * @return YAML::Node
   */
  inline YAML::Node retrieve_param_node(const YAML::Node &config, const std::string &name)
  {
    // !!! Note: Must use Clone()
    // YAML::Node params = config is a simple alias, not a copy
    YAML::Node params = YAML::Clone(config);
    if (!name.empty())
    {
      std::vector<std::string> tokens = split(name, '/');
      for (auto name : tokens)
      {
        if (name.empty())
          continue;
        params = params[name];
      }
    }

    return params;
  }

  /**
   * @brief find all sub namespaces unter a parent namespace
   *
   * @example for parent_ns="/robot/frames" returns "torso"
   *  robot:
   *    frames:
   *      torso:
   *        pos: [...]
   *
   * @param config
   * @param parent_ns namespaces of every layer from the top level
   * @return std::vector<std::string>
   */
  inline std::vector<std::string> find_sub_namespaces(const YAML::Node config, const std::string &parent_ns = "")
  {
    std::vector<std::string> sub_namespaces;

    YAML::Node params = YAML::Clone(config);
    if (!parent_ns.empty())
    {
      std::vector<std::string> tokens = split(parent_ns, '/');
      for (auto name : tokens)
      {
        if (name.empty())
          continue;

        params = params[name];
      }
    }

    if (params.Type() == YAML::NodeType::Map)
    {
      for (auto it = params.begin(); it != params.end(); it++)
      {
        std::string name = it->first.as<std::string>();
        sub_namespaces.push_back(name);
      }
    }

    return sub_namespaces;
  }

  /**
   * @brief Load parameter from the config node
   *
   * @tparam T
   * @param config config node loaded from yaml
   * @param name parameter name
   * @param val default/loaded value
   * @param is_required required or optional
   * @return true
   * @return false
   */
  template <typename T>
  inline bool load(const YAML::Node &config, const std::string &name, T &val,
                   bool is_required = true)
  {
    YAML::Node node;
    try
    {
      node = yaml::retrieve_param_node(config, name);
      // type convertion
      val = node.as<T>();
    }
    catch (const std::exception &e)
    {
      // check if not found or wrong type
      if (node.IsDefined())
      {
        if (is_required)
        {
          PRINT_ERROR("Required parameter [" << name << "] found but have wrong type");
          return false;
        }
        else
          PRINT_WARN("Optional parameter [" << name << "] found but have wrong type, Using default value: ");
      }
      else
      {
        if (is_required)
        {
          PRINT_ERROR("Required parameter [" << name << "] not found");
          return false;
        }
        else
          PRINT_WARN("Optional parameter [" << name << "] not found, Using default value");
      }
    }
    return true;
  }
}