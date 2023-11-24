// *********************************************************
//
// File autogenerated for the tomm_basic_behaviors package
// by the rosparam_handler package.
// Please do not edit.
//
// ********************************************************/

#pragma once

#include <ros/param.h>
#include <control_core/ros/parameters.h>
#include <rosparam_handler/utilities_std.h>
#include <control_core/types.h>
#ifdef DYNAMIC_RECONFIGURE_FOUND
#include <tomm_basic_behaviors/MoveBaseConfig.h>
#include <dynamic_reconfigure/server.h>
namespace tomm_basic_behaviors
{
    typedef dynamic_reconfigure::Server<MoveBaseConfig> MoveBaseServer;
}
#else
namespace tomm_basic_behaviors
{
    struct MoveBaseConfig{};
    struct MoveBaseServer{};
}
#endif


namespace tomm_basic_behaviors {

/// \brief Parameter struct generated by rosparam_handler
class MoveBaseParameters : public cc::Parameters {

public:
  typedef cc::Parameters Base;
  using Config = MoveBaseConfig;

public:
  MoveBaseParameters()
  : Base(), config_updated_once_(false)
  {
    addToList();
  }

  MoveBaseParameters(const ros::NodeHandle& private_node_handle, const std::string& private_namespace="")
  : Base(private_node_handle, private_namespace), config_updated_once_(false) 
  {
    addToList();
  }

  ~MoveBaseParameters() { }

  /// \brief Get values from parameter server
  /// 
  /// Will fail if a value can not be found and no default value is given.
  bool fromParamServer() {
    bool success = true;
    success &= utilities_std::getParam(private_namespace_ + "topic", topic);

    if(!success){
      missingParamsWarning();
      ROS_ERROR("RosparamHandler: GetParam could not retrieve parameter.");
    }
    ROS_DEBUG_STREAM(*this);
    is_loaded_ = true;
    return success;
  }

  /// \brief Get values from parameter server
  /// 
  /// Will fail if a value can not be found and no default value is given.
  bool fromParamServer(const ros::NodeHandle& private_node_handle, const std::string& private_namespace="") {
    private_namespace_ = rosparam_handler::joinNamespaces({private_node_handle.getNamespace(), private_namespace});
    node_name_ = rosparam_handler::getNodeName(private_node_handle);
    is_init_ = true;
    return fromParamServer();
  }

  /// \brief Set parameters on ROS parameter server.
  bool toParamServer() {
  utilities_std::setParam(private_namespace_ + "topic",topic);
    return true;        
  }

  /// \brief Update configurable parameters.
  ///
  /// \param config  dynamic reconfigure struct
  /// \level ?
  void fromConfig(const Config& config, const uint32_t level = 0) {
#ifdef DYNAMIC_RECONFIGURE_FOUND

#else
  ROS_FATAL_STREAM("dynamic_reconfigure was not found during compilation. So fromConfig() is not available. Please recompile with dynamic_reconfigure.");
#endif
  }

  void toConfig(Config& config) {
#ifdef DYNAMIC_RECONFIGURE_FOUND
    
#else
      ROS_FATAL_STREAM("dynamic_reconfigure was not found during compilation. So toConfig() is not available. Please recompile with dynamic_reconfigure.");
#endif
  }

  Config asConfig() const {
    Config config;
#ifdef DYNAMIC_RECONFIGURE_FOUND
    
#else
      ROS_FATAL_STREAM("dynamic_reconfigure was not found during compilation. So toConfig() is not available. Please recompile with dynamic_reconfigure.");
#endif
    return config;
  }

  /// \brief Update configurable parameters.
  ///
  /// \param config  dynamic reconfigure struct
  /// \level ?
  void updateConfig(Config& config, const uint32_t level = 0) {
#ifdef DYNAMIC_RECONFIGURE_FOUND
  if(!config_updated_once_)
  {
    // first update, override config
    config_updated_once_ = true;
    toConfig(config);
  }
  else
  {
    // subsequent update, read from config
    fromConfig(config, level);
  }
#else
  ROS_FATAL_STREAM("dynamic_reconfigure was not found during compilation. So updateConfig() is not available. Please recompile with dynamic_reconfigure.");
#endif
  }

  /// \brief Stream operator for printing parameter struct
  friend std::ostream& operator<<(std::ostream& os, const MoveBaseParameters& p)
  {
    os << "[" << p.node_name_ << "]\nNode " << p.node_name_ << " has the following parameters:\n"
      << "	" << p.private_namespace_ << "topic:" << p.topic << "\n"
;
    return os;
  }

public:
    std::string topic; /*!<  */

private:
  bool config_updated_once_;
  /// \brief Add them to the parmeter list
  void addToList() {
        Base::addReference("topic", topic);
  }

  /// \brief Issue a warning about missing default parameters.
  void missingParamsWarning(){
    ROS_WARN_STREAM("[" << node_name_ << "]\nThe following parameters do not have default values and need to be specified:\n"
      << "	" << private_namespace_ << "topic" << " (std::string) \n"
    );
  }
};

} // namespace tomm_basic_behaviors
