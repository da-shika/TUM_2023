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

#ifndef CONTROL_CORE_ROS_ROS_H
#define CONTROL_CORE_ROS_ROS_H

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <control_core/types.h>
#include <control_core/utilities/utilities.h>

namespace cc
{

  /**
   * @brief check if topic of given name and type is published
   * 
   * @param name name of topic with full path
   * @param type message type 
   * @return true 
   * @return false 
   */
  inline bool is_topic_published(
    const std::string& name, 
    const std::string& message_type = "")
  {
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    for(auto info : master_topics)
    {
      if(message_type.empty())
      {
        if(info.name == name)
        {
          return true;
        }
      }
      else
      {
        if(info.name == name && info.datatype == message_type)
        {
          return true;
        }
      }
    }
    return false;
  }

  inline bool is_service_available(
    const std::string& name)
  {
    return ros::service::exists(name, false);
  }

  inline bool wait_for_service(const std::string &service_name, ros::Duration timeout=((ros::Duration)(-1)), ros::Duration print_time=((ros::Duration)(-1)))
  {
    if(print_time.toSec() < -1)
    {
      return ros::service::waitForService(service_name, timeout);
    }
    else
    {
      int n_trials = std::min(1, int(std::ceil(timeout.toSec()/print_time.toSec())));
      for(int i = 0; i < n_trials; ++i)
      {
        if(ros::service::waitForService(service_name, print_time))
          return true;
        ROS_INFO("wait_for_service: '%s' trial=%d/%d", service_name.c_str(), i, n_trials);
      }
      return false;
    }
  }

  /*!
  * \brief Publish a msg if there is a subsriber
  */
  template <typename _Msg>
  inline bool publish_if_subscribed(ros::Publisher &pub, const _Msg &msg)
  {
    if (pub.getNumSubscribers())
    {
      pub.publish(msg);
      return true;
    }
    return false;
  }

  /**
   * @brief create a tf stamped transformation from pose
   * 
   */
  inline tf::StampedTransform make_transform(
    const cc::CartesianPosition& pose, 
    const std::string& parent, const std::string& child,
    const ros::Time& time=ros::Time::now())
  {
    tf::StampedTransform transf;
    transf.setData(pose.toTransformTf());
    transf.stamp_ = time;
    transf.frame_id_ = parent;
    transf.child_frame_id_ = child;
    return transf;
  }

  inline bool broadcastTransformation(
    tf::TransformBroadcaster& broadcaster,
    const cc::CartesianPosition& pose, 
    const std::string& parent, const std::string& child,
    const ros::Time& time=ros::Time::now())
  {
    broadcaster.sendTransform(make_transform(pose, parent, child, time));
    return true;
  }

  /*!
  * \brief Read a transformation
  */
  inline bool listenTransformation(
      tf::TransformListener &listener,
      const std::string &parent,
      const std::string &child,
      CartesianPosition &X_parent_child,
      Scalar dur = 10.0,
      bool verbose = true)
  {
    if (parent == child)
    {
      X_parent_child.setIdentity();
      return true;
    }

    tf::StampedTransform transform;
    try
    {
      listener.waitForTransform(child, parent, ros::Time(0), ros::Duration(dur));
      listener.lookupTransform(parent, child, ros::Time(0), transform);
    }
    catch (const tf::TransformException &ex)
    {
      if (verbose)
        ROS_ERROR_STREAM("listenTransformation(): Can't find transformation " << child << " w.r.t. " << parent);
      return false;
    }
    X_parent_child = transform;
    return true;
  }

  /*!
  * \brief Check if ros has parameter.
  */
  inline bool has(const std::string &name, bool verbose = true)
  {
    if (!ros::param::has(name))
    {
      if (verbose)
      {
        ROS_ERROR("has: parameter '%s' is missing.", name.c_str());
      }
      return false;
    }
    return true;
  }

  /**
   * @brief Get the name of a node handle
   * 
   * @param private_node_handle 
   * @return std::string 
   */
  inline std::string get_node_name(const ros::NodeHandle& private_node_handle) 
  {
    std::string name_space = private_node_handle.getNamespace();
    std::stringstream tempString(name_space);
    std::string name;
    while(std::getline(tempString, name, '/')) {;}
    return name;
  }

  /**
   * @brief joint multiple strings into valid namespace
   * 
   * @param ns_names 
   * @return std::string 
   */
  inline std::string join_namespaces(const std::vector<std::string>& ns_names)
  {
    if(ns_names.empty())
      return "/";
    
    std::string ns_joined;
    for(const auto& ns : ns_names)
    {
      if(!ns.empty())
      {
        ns_joined += ns + "/";
      }
    }
    return ns_joined;
  }

  /**
   * @brief returns all sub name spaces below given searched_ns
   * 
   * @example find_sub_names_spaces("frames")
   * returns "torso" for an existing parameter "robot/frames/torso/pos" on the server
   *
   */
  inline std::vector<std::string> find_sub_names_spaces(const std::string& searched_ns, const std::string& parent_ns="")
  {
    std::vector<std::string> sub_name_spaces;
    std::vector<std::string> names;

    // read all parameters
    ros::param::getParamNames(names);
    for(std::size_t i = 0; i < names.size(); ++i)
    {
      // split into ns
      const std::string& name = names[i];
      std::vector<std::string> tokens = split(name, '/');

      // check if in parent_ns
      if(!parent_ns.empty() && tokens.size() > 2)
      {
        if(tokens[1] != parent_ns)
          continue;
      }
      for(std::size_t j = 0; j < tokens.size() - 1; ++j)
      {
        // check if ns matches search
        const std::string& ns = tokens[j];
        const std::string& sub_ns = tokens[j+1];
        if(searched_ns == ns)
        {
          // add sub namesspace
          if(!cc::has(sub_name_spaces, sub_ns))
          {
            sub_name_spaces.push_back(sub_ns);
          }
        }
      }
    }
    return sub_name_spaces;
  }

  /*!
  * \brief load bool value
  */
  inline bool load(const std::string &name, bool &val, bool verbose = true)
  {
    if (!has(name, verbose))
    {
      return false;
    }
    if (!ros::param::get(name, val))
    {
      if (verbose)
      {
        ROS_ERROR("load: could not load '%s'.", name.c_str());
      }
      return false;
    }
    return true;
  }

  /*!
  * \brief load int value
  */
  inline bool load(const std::string &name, int &val, bool verbose = true)
  {
    if (!has(name, verbose))
    {
      return false;
    }
    if (!ros::param::get(name, val))
    {
      if (verbose)
      {
        ROS_ERROR("load: could not load '%s'.", name.c_str());
      }
      return false;
    }
    return true;
  }

  /*!
  * \brief load unsigned int value
  */
  inline bool load(const std::string &name, size_t &val, bool verbose = true)
  {
    int val_;
    if (!load(name, val_, verbose))
    {
      return false;
    }
    if (val_ < 0)
    {
      if (verbose)
      {
        ROS_ERROR("load: could not load '%s'.", name.c_str());
      }
      return false;
    }
    val = val_;
    return true;
  }

  /*!
  * \brief load scalar value
  */
  inline bool load(const std::string &name, cc::Scalar &val, bool verbose = true)
  {
    if (!has(name, verbose))
    {
      return false;
    }
    if (!ros::param::get(name, val))
    {
      if (verbose)
      {
        ROS_ERROR("load: could not load '%s'.", name.c_str());
      }
      return false;
    }
    return true;
  }

  /*!
  * \brief load string value
  */
  inline bool load(const std::string &name, std::string &val, bool verbose = true)
  {
    if (!has(name, verbose))
    {
      return false;
    }
    if (!ros::param::get(name, val))
    {
      if (verbose)
      {
        ROS_ERROR("load: could not load '%s'.", name.c_str());
      }
      return false;
    }
    return true;
  }

  /*!
  * \brief load an array of string values
  */
  inline bool load(const std::string &name, std::vector<std::string>& vals, bool verbose = true)
  {
    if (!has(name, verbose))
    {
      return false;
    }
    XmlRpc::XmlRpcValue items;
    if (!ros::param::get(name, items))
    {
      if (verbose)
      {
        ROS_ERROR("load: could not load '%s'.", name.c_str());
      }
      return false;
    }
    if (items.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      if (verbose)
      {
        ROS_ERROR("load: value '%s' not a string array.", name.c_str());
      }
      return false;
    }
    vals.clear();
    for (int i = 0; i < items.size(); ++i)
    {
      XmlRpc::XmlRpcValue& item = items[i];
      if (item.getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        if (verbose)
        {
          ROS_ERROR("load: value '%s' not a string array.", name.c_str());
        }
        return false;
      }
      vals.push_back(static_cast<std::string>(item));
    }
    return true;
  }

  /*!
  * \brief load std::vector value.
  */
  inline bool load( const std::string &name, std::vector<cc::Scalar> &val,
    bool verbose = true)
  {
    if (!has(name, verbose))
    {
      return false;
    }
    if (!ros::param::get(name, val))
    {
      if (verbose)
      {
        ROS_ERROR("load: could not load std::vector '%s'.", name.c_str());
      }
      return false;
    }
    return true;
  }

  /*!
  * \brief load generic eigen matrix value.
  * If fixed size: loaded dimension must match
  * If variable size and rows and cols specified: used as size
  * Else resized by loaded param.
  */
  template <typename _Derived>
  inline bool load(const std::string &name, _Derived &val,
    bool verbose = true)
  {
    typedef typename _Derived::Index Index;

    Index Rows, Cols, Size;
    Rows = _Derived::RowsAtCompileTime;
    Cols = _Derived::ColsAtCompileTime;
    Size = _Derived::SizeAtCompileTime;

    std::vector<cc::Scalar> v;
    if (Rows == Eigen::Dynamic || Cols == Eigen::Dynamic)
    {
      // // load the vector
      if (!load(name, v, verbose))
      {
        if(verbose)
        {
          ROS_ERROR("load: could not load Eigen::Matrix '%s'.", name.c_str());
        }
        return false;
      }
      else if (Rows == Eigen::Dynamic && Cols != Eigen::Dynamic)
      {
        // rows are free
        val.resize(v.size(), Cols);
      }
      else if (Cols == Eigen::Dynamic && Rows != Eigen::Dynamic)
      {
        // cols are free
        val.resize(Rows, v.size());
      }
      else
      {
        // default case, both are free, make nx1 dim vector
        val.resize(v.size(), 1);
      }
    }
    else
    {
      // matrix is fixed size, check dimension
      if (!load(name, v, verbose))
      {
        if(Index(v.size()) != Size)
        {
          if(verbose)
          {
            ROS_ERROR("load: could not load Eigen::Matrix '%s', because loaded has dim=%ld != desired=%ld", 
              name.c_str(), v.size(), Size);
          }
          return false;
        }
        if(verbose)
        {
          ROS_ERROR("load: could not load Eigen::Matrix '%s'.", name.c_str());
        }
        return false;
      }
    }

    // store data
    val.setZero();
    for (Index i = 0; i < val.size(); ++i)
    {
      val(i) = v[i];
    }
    return true;
  }

}

#endif // CONTROL_CORE_ROS_H