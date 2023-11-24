// *********************************************************
//
// File autogenerated for the joy_target package
// by the rosparam_handler package.
// Please do not edit.
//
// ********************************************************/

#pragma once

#include <ros/param.h>
#include <control_core/ros/parameters.h>
#include <rosparam_handler/utilities_std.h>
#include <control_core/types.h>
#include <rosparam_handler/utilities_eigen.h>
#ifdef DYNAMIC_RECONFIGURE_FOUND
#include <joy_target/MarkerTargetConfig.h>
#include <dynamic_reconfigure/server.h>
namespace joy_target
{
    typedef dynamic_reconfigure::Server<MarkerTargetConfig> MarkerTargetServer;
}
#else
namespace joy_target
{
    struct MarkerTargetConfig{};
    struct MarkerTargetServer{};
}
#endif


namespace joy_target {

/// \brief Parameter struct generated by rosparam_handler
class MarkerTargetParameters : public cc::Parameters {

public:
  typedef cc::Parameters Base;
  using Config = MarkerTargetConfig;

public:
  MarkerTargetParameters()
  : Base(), config_updated_once_(false)
  {
    addToList();
  }

  MarkerTargetParameters(const ros::NodeHandle& private_node_handle, const std::string& private_namespace="")
  : Base(private_node_handle, private_namespace), config_updated_once_(false) 
  {
    addToList();
  }

  ~MarkerTargetParameters() { }

  /// \brief Get values from parameter server
  /// 
  /// Will fail if a value can not be found and no default value is given.
  bool fromParamServer() {
    bool success = true;
    success &= utilities_std::getParam(private_namespace_ + "topic_pre_fix", topic_pre_fix, std::string{""});
    success &= utilities_std::getParam(private_namespace_ + "tf_pre_fix", tf_pre_fix, std::string{""});
    success &= utilities_std::getParam(private_namespace_ + "parent_frame", parent_frame);
    success &= utilities_std::getParam(private_namespace_ + "frames", frames);
    success &= utilities_eigen::getParam(private_namespace_ + "mass", mass);
    success &= utilities_eigen::getParam(private_namespace_ + "spring", spring);
    success &= utilities_eigen::getParam(private_namespace_ + "damping", damping);
    success &= utilities_eigen::getParam(private_namespace_ + "acc_lb", acc_lb, std::vector<double>{-100,-100,-100,-100,-100,-100,-100});
    success &= utilities_eigen::getParam(private_namespace_ + "acc_ub", acc_ub, std::vector<double>{100,100,100,100,100,100,100});
    success &= utilities_eigen::getParam(private_namespace_ + "vel_lb", vel_lb, std::vector<double>{-100,-100,-100,-100,-100,-100,-100});
    success &= utilities_eigen::getParam(private_namespace_ + "vel_ub", vel_ub, std::vector<double>{100,100,100,100,100,100,100});

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
  utilities_std::setParam(private_namespace_ + "topic_pre_fix",topic_pre_fix);
  utilities_std::setParam(private_namespace_ + "tf_pre_fix",tf_pre_fix);
  utilities_std::setParam(private_namespace_ + "parent_frame",parent_frame);
  utilities_std::setParam(private_namespace_ + "frames",frames);
  utilities_eigen::setParam(private_namespace_ + "mass",mass);
  utilities_eigen::setParam(private_namespace_ + "spring",spring);
  utilities_eigen::setParam(private_namespace_ + "damping",damping);
  utilities_eigen::setParam(private_namespace_ + "acc_lb",acc_lb);
  utilities_eigen::setParam(private_namespace_ + "acc_ub",acc_ub);
  utilities_eigen::setParam(private_namespace_ + "vel_lb",vel_lb);
  utilities_eigen::setParam(private_namespace_ + "vel_ub",vel_ub);
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
  friend std::ostream& operator<<(std::ostream& os, const MarkerTargetParameters& p)
  {
    os << "[" << p.node_name_ << "]\nNode " << p.node_name_ << " has the following parameters:\n"
      << "	" << p.private_namespace_ << "topic_pre_fix:" << p.topic_pre_fix << "\n"
      << "	" << p.private_namespace_ << "tf_pre_fix:" << p.tf_pre_fix << "\n"
      << "	" << p.private_namespace_ << "parent_frame:" << p.parent_frame << "\n"
      << "	" << p.private_namespace_ << "frames:" << utilities_std::to_string(p.frames) << "\n"
      << "	" << p.private_namespace_ << "mass:" << utilities_eigen::to_string(p.mass) << "\n"
      << "	" << p.private_namespace_ << "spring:" << utilities_eigen::to_string(p.spring) << "\n"
      << "	" << p.private_namespace_ << "damping:" << utilities_eigen::to_string(p.damping) << "\n"
      << "	" << p.private_namespace_ << "acc_lb:" << utilities_eigen::to_string(p.acc_lb) << "\n"
      << "	" << p.private_namespace_ << "acc_ub:" << utilities_eigen::to_string(p.acc_ub) << "\n"
      << "	" << p.private_namespace_ << "vel_lb:" << utilities_eigen::to_string(p.vel_lb) << "\n"
      << "	" << p.private_namespace_ << "vel_ub:" << utilities_eigen::to_string(p.vel_ub) << "\n"
;
    return os;
  }

public:
    std::string topic_pre_fix; /*!<  */
  std::string tf_pre_fix; /*!<  */
  std::string parent_frame; /*!<  */
  std::vector<std::string> frames; /*!<  */
  cc::Vector6 mass; /*!<  */
  cc::Vector6 spring; /*!<  */
  cc::Vector6 damping; /*!<  */
  cc::Vector6 acc_lb; /*!<  */
  cc::Vector6 acc_ub; /*!<  */
  cc::Vector6 vel_lb; /*!<  */
  cc::Vector6 vel_ub; /*!<  */

private:
  bool config_updated_once_;
  /// \brief Add them to the parmeter list
  void addToList() {
        Base::addReference("topic_pre_fix", topic_pre_fix);
    Base::addReference("tf_pre_fix", tf_pre_fix);
    Base::addReference("parent_frame", parent_frame);
    Base::addReference("frames", frames);
    Base::addReference("mass", mass);
    Base::addReference("spring", spring);
    Base::addReference("damping", damping);
    Base::addReference("acc_lb", acc_lb);
    Base::addReference("acc_ub", acc_ub);
    Base::addReference("vel_lb", vel_lb);
    Base::addReference("vel_ub", vel_ub);
  }

  /// \brief Issue a warning about missing default parameters.
  void missingParamsWarning(){
    ROS_WARN_STREAM("[" << node_name_ << "]\nThe following parameters do not have default values and need to be specified:\n"
      << "	" << private_namespace_ << "parent_frame" << " (std::string) \n"
      << "	" << private_namespace_ << "frames" << " (std::vector<std::string>) \n"
      << "	" << private_namespace_ << "mass" << " (cc::Vector6) \n"
      << "	" << private_namespace_ << "spring" << " (cc::Vector6) \n"
      << "	" << private_namespace_ << "damping" << " (cc::Vector6) \n"
    );
  }
};

} // namespace joy_target
