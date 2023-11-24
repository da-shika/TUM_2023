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


#ifndef CONTROL_CORE_PARAMETERS_H
#define CONTROL_CORE_PARAMETERS_H

#include <control_core/ros/ros.h>
#include <memory>
#include <unordered_map>

namespace cc
{

  /**
   * @brief Entry holds parameter information
   * 
   */
  class Entry
  {
  public:
    std::string name_;
    bool is_required_;
    bool is_loaded_;
    bool is_ref_;

  public:
    Entry(
        const std::string& name, 
        bool is_required=true, 
        bool is_loaded=false,
        bool is_ref=false) :
      name_(name),
      is_required_(is_required),
      is_loaded_(is_loaded),
      is_ref_(is_ref)
    {
    }

    virtual ~Entry()
    {
    }

    virtual bool load(const std::string& private_namespace) = 0;
  };

  /**
   * @brief TypedEntry hold a value or reference to a parameter
   * 
   * @tparam T 
   */
  template<typename T>
  class TypedEntry : public Entry 
  {
  public:
    T val_;   // can be a reference or a value

    /**
     * @brief Construct without actual content
     * 
     * @param name 
     */
    TypedEntry(const std::string& name) :
      Entry(name, true, false, false)
    {
    } 

    /**
     * @brief Construct with default content
     * 
     * @param name 
     * @param val 
     * @param is_required optional or requried
     * @param is_loaded loaded or not loaded
     */
    TypedEntry(const std::string& name, T val, bool is_required=true, bool is_loaded=false, bool is_ref=false) :
      Entry(name, is_required, is_loaded, is_ref),
      val_{val}
    {
    }

    virtual ~TypedEntry()
    {
    }

    /**
     * @brief load form given namespace
     * 
     * @param ns 
     * @return true 
     * @return false 
     */
    bool load(const std::string& private_namespace)
    {
      bool verbose = true;
      if(!is_required_)
      {
        verbose = false;
      }
      is_loaded_ = cc::load(private_namespace + name_, val_, verbose);
      return is_loaded_;
    }
  };

  /**
   * @brief ParametersBase
   * 
   * The baseclass for all parameters. 
   * This allows to access parameters of the derived class 
   * through functions like has() get(). 
   * 
   * Note: This requries a dynamic_cast<>, but is useful when the derived class
   * object is not available.
   */
  class Parameters
  {
    public:
      typedef std::unordered_map<std::string, std::shared_ptr<Entry> > Entries;
      typedef Entries::iterator EntriesIter;
      typedef Entries::const_iterator CEntriesIter;

    protected:
      bool is_loaded_;
      bool is_init_;
      std::string global_namespace_;
      std::string private_namespace_;
      std::string node_name_;
      Entries entries_;

    public:
      Parameters() : 
        is_loaded_(false),
        is_init_(false),
        global_namespace_{"/"} 
      {
      }

      /**
       * @brief Parameters Base 
       * 
       * Namespace is concat of node_handle namespace and private_namespace 
       *
       * @param private_node_handle 
       * @param private_namespace 
       */
      Parameters(const ros::NodeHandle& private_node_handle, const std::string& private_namespace="") : 
        is_loaded_(false),
        is_init_(true),
        global_namespace_{"/"},
        private_namespace_{join_namespaces({private_node_handle.getNamespace(), private_namespace})},
        node_name_{get_node_name(private_node_handle)} 
      {
      }

      virtual ~Parameters()
      {
      }

      /**
       * @brief check if parameters where loaded at least once
       * 
       * @return true 
       * @return false 
       */
      bool isLoaded() const
      {
        return is_loaded_;
      }      

      /**
       * @brief get the namespace parameters are loaded from
       * 
       * @return std::string 
       */
      std::string privateNamespace() const
      {
        return private_namespace_;
      }

      /**
       * @brief load from given namespace
       * 
       * @param ns 
       * @return true 
       * @return false 
       */
      bool load(const std::string& private_namespace)
      {
        private_namespace_ = private_namespace;
        is_init_ = true;
        return load();
      }

      /**
       * @brief load from concat of node and private_namespace
       * 
       * @param private_node_handle 
       * @param private_namespace 
       * @return true 
       * @return false 
       */
      bool load(const ros::NodeHandle& private_node_handle, const std::string& private_namespace="")
      {
        private_namespace_ = join_namespaces({private_node_handle.getNamespace(), private_namespace});
        node_name_ = get_node_name(private_node_handle);
        is_init_ = true;
        return load();
      }

      /**
       * @brief loads parameters from server
       * 
       * @return true 
       * @return false 
       */
      bool load()
      {
        // load all non reference parameters
        bool ret = true;
        for(EntriesIter it = entries_.begin(); it != entries_.end(); ++it)
        {
          if(it->second->is_required_)
            ret &= it->second->load(private_namespace_);
          else
            it->second->load(private_namespace_);
        }
        
        // load remaining derived parameters
        ret &= fromParamServer();

        // check if okay
        if(ret)
        {
          is_loaded_ = true;
        }
        else
        {
          is_loaded_ = false;
          ROS_ERROR("Parameter::load(): Failed loading parameter from ns='%s'", 
            private_namespace_.c_str());
        }
        return ret;
      }

      /**
       * @brief add an optional parameter with given default value
       * 
       * @tparam T 
       * @param name 
       * @param val 
       * @return true 
       * @return false 
       */
      template<typename T>
      bool addOptional(const std::string& name, const T& val)
      {
        EntriesIter it = entries_.find(name);
        if(it != entries_.end())
        {
          ROS_WARN("Parameter add: Name '%s' already exsists, overwriting", name.c_str());
          it->second = std::make_shared<TypedEntry<T> >(name, val, false);
        }
        else
        {
          entries_[name] = std::make_shared<TypedEntry<T> >(name, val, false);
        }
        return true;
      }

      template<typename T>
      bool addOptional(const std::vector<std::string>& names, const T& val)
      {
        bool ret = true;
        for(size_t i = 0; i < names.size(); ++i)
          ret &= addOptional<T>(names[i], val);
        return ret;
      }

      /**
       * @brief add a requried value
       * 
       * @tparam T 
       * @param name 
       * @return true 
       * @return false 
       */
      template<typename T>
      bool addRequired(const std::string& name)
      {
        EntriesIter it = entries_.find(name);
        if(it != entries_.end())
        {
          ROS_WARN("Parameter add: Name '%s' already exsists, overwriting", name.c_str());
          it->second = std::make_shared<TypedEntry<T> >(name);
        }
        else
        {
          entries_[name] = std::make_shared<TypedEntry<T> >(name);
        }
        return true;
      }

      template<typename T>
      bool addRequired(const std::vector<std::string>& names)
      {
        bool ret = true;
        for(size_t i = 0; i < names.size(); ++i)
          ret &= addRequired<T>(names[i]);
        return ret;
      }

      /**
       * @brief check if parameter added
       * 
       * @tparam T 
       * @param name 
       * @return true 
       * @return false 
       */
      template<typename T> 
      bool has(const std::string& name)
      {
        CEntriesIter it = entries_.find(name);
        if(it == entries_.end())
        {
          ROS_ERROR("ParametersBase::get(): Name '%s' not found.", name.c_str());
          return false;
        }
        return (std::dynamic_pointer_cast<TypedEntry<T> >(it->second) != nullptr);
      }

      /**
       * @brief check if parameter loaded
       * 
       * @param name 
       * @return true loaded from parameter server
       * @return false not loaded, using default
       */
      bool isLoaded(const std::string& name)
      {
        CEntriesIter it = entries_.find(name);
        if(it == entries_.end())
        {
          ROS_ERROR("Parameter get: Name '%s' not found.", name.c_str());
          return false;
        }
        return it->second->is_loaded_;
      }

      /**
       * @brief gets the parameter and copies its value
       * 
       * @tparam T 
       * @param name 
       * @param val 
       * @return true 
       * @return false 
       */
      template<typename T>
      bool get(const std::string& name, T& val) const
      {
        if(!is_loaded_)
        {
          ROS_ERROR("ParametersBase::get(): has never been loaded, call load() frist.");
          return false;
        }

        CEntriesIter it = entries_.find(name);
        if(it == entries_.end())
        {
          ROS_ERROR("ParametersBase::get(): Name '%s' not found.", name.c_str());
          return false;
        }

        std::shared_ptr<TypedEntry<T> > ptr = 
          std::dynamic_pointer_cast<TypedEntry<T> >(it->second);
        if(ptr)
        {
          val = ptr->val_;
          return true;
        }
        std::shared_ptr<TypedEntry<T&> > ptr_ref = 
          std::dynamic_pointer_cast<TypedEntry<T&> >(it->second);
        if(ptr_ref)
        {
          val = ptr_ref->val_;
          return true;
        }

        ROS_ERROR("ParametersBase::get(): Name '%s' found, but wrong template type.", name.c_str());
        return false;
      }

      /**
       * @brief get a copy of given parameter name
       * 
       * @tparam T 
       * @param name 
       * @return T 
       */
      template<typename T>
      T get(const std::string& name) const
      {
        T val = T();
        get(name, val);
        return val;
      }

      std::string toString()
      {
        std::ostringstream out;
        for(EntriesIter it = entries_.begin(); it != entries_.end(); ++it)
          out << " - " << it->first << std::endl;
        return out.str();
      }

      virtual bool fromParamServer()
      {
        // default: nothing to do
        return true;
      }

      virtual bool toParamServer()
      {
        ROS_ERROR("ParametersBase::toParamServer() not overloaded");
        return false;
      }

    protected:
      template<typename T>
      bool addReference(const std::string& name, T && val)
      {
        entries_[name] = std::make_shared<TypedEntry<T> >(name, val, false, true, std::is_reference<T>::value);
        return true;
      }
  };

}

#endif // CONTROL_CORE_PARAMETERS_H
