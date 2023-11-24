#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <yaml_parameters/utilities.h>

namespace yaml
{
  /**
   * @brief Parameters
   *
   * The baseclass for all parameters, loaded from a yaml file.
   *
   */
  class Parameters
  {
  protected:
    bool is_loaded_;
    YAML::Node config_;
    std::string ns_;

  public:
    /**
     * @brief Parameters Base
     *
     * Namespace is concat of node_handle namespace and private_namespace
     *
     * @param ns namespace
     */
    Parameters(const std::string &ns = "") : is_loaded_(false),
                                             ns_{ns}
    {
      check_ns();
    }

    virtual ~Parameters()
    {
    }

    /**
     * @brief check if yaml file loaded
     *
     * @return true
     * @return false
     */
    bool isLoaded() const
    {
      return is_loaded_;
    }

    /**
     * @brief Namespace getter
     *
     * @return std::string
     */
    std::string ns() const
    {
      return ns_;
    }

    /**
     * @brief Namespace setter
     *
     * @param ns
     */
    void set_ns(const std::string &ns)
    {
      ns_ = ns;
      check_ns();
    }

    /**
     * @brief loads yaml file from given path
     *
     * @param file_path yaml file path
     * @return true
     * @return false
     */
    bool loadFile(const std::string &file_path)
    {
      try
      {
        config_ = YAML::LoadFile(file_path);
      }
      catch (const std::exception &e)
      {
        PRINT_ERROR("ConfigBase::load(): Failed to load config file " << file_path);
        return false;
      }
      is_loaded_ = true;
      return true;
    }

    /**
     * @brief check if parameter exist
     *
     * @param name
     * @return true
     * @return false
     */
    bool has(const std::string &name)
    {
      YAML::Node node = yaml::retrieve_param_node(config_, ns_ + name);
      return node.IsDefined();
    }

    /**
     * @brief Get the Required parameter
     *
     * @tparam T
     * @param name parameter name
     * @param val
     * @return true
     * @return false
     */
    template <typename T>
    bool get(const std::string &name, T &val) const
    {
      return load(config_, ns_ + name, val, true);
    }

    /**
     * @brief Get the Required parameter
     *
     * @tparam T
     * @param name
     * @return T
     */
    template <typename T>
    T get(const std::string &name) const
    {
      T val = T();
      get(name, val);
      return val;
    }

    /**
     * @brief Get the Optional parameter, use default value if not found or wrong type
     *
     * @tparam T
     * @param name
     * @param val
     * @return true
     * @return false
     */
    template <typename T>
    bool get(const std::string &name, const T &default_val, T &val) const
    {
      val = default_val;
      return load(config_, ns_ + name, val, false);
    }

    template <typename T>
    T get(const std::string &name, const T &default_val) const
    {
      T val = T();
      get(name, default_val, val);
      return val;
    }

    /**
     * @brief Get sub-namespaces
     *
     * @example return {"h1", "tomm"} for searched_ns="robot"
     *  robot:
     *    h1: sim
     *    tomm: sim
     *
     * @param searched_ns can be with or without '/' in front
     * @return std::vector<std::string>
     */
    std::vector<std::string> getSubNamespaces(const std::string &searched_ns)
    {
      std::string parent_ns;

      // append ns_ in front
      if (searched_ns.front() == '/')
      {
        // remove the '/' in front of searched_ns
        // because ns_ already contains '/' in the end
        parent_ns = ns_ + searched_ns.substr(1, searched_ns.size() - 1);
      }
      else
      {
        parent_ns = ns_ + searched_ns;
      }
      return find_sub_namespaces(config_, parent_ns);
    }

    /**
     * @brief Get root namespaces
     *
     * @return std::vector<std::string>
     */
    std::vector<std::string> getRootNamespaces()
    {
      return find_sub_namespaces(config_, "");
    }

  private:
    /**
     * @brief Check if ns_ ends with '/', if no, add one
     *
     */
    void check_ns()
    {
      if (!ns_.empty() && ns_.back() != '/')
      {
        ns_ += '/';
      }
    }
  };
}
