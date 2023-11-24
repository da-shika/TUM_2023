#include <tomm_control_plugin/control_plugin_base.h>

using namespace std;
using namespace hardware_interface;

namespace tomm_control_plugin
{

  TOMMControlPluginBase::TOMMControlPluginBase(bool claim_resources) : 
    claim_resources_(claim_resources)
  {                                         
  }

  TOMMControlPluginBase::~TOMMControlPluginBase()
  {
  }

  bool TOMMControlPluginBase::initRequest(hardware_interface::RobotHW *robot_hw,
                                       ros::NodeHandle &root_nh,
                                       ros::NodeHandle &controller_nh,
                                       ClaimedResources &claimed_resources)
  {
    name_ = controller_nh.getNamespace();

    ////////////////////////////////////////////////////////////////////////////
    // Setup handles
    ////////////////////////////////////////////////////////////////////////////

    // Get a pointer to the joint position control interface
    PosVelJointInterface *pos_iface = robot_hw->get<PosVelJointInterface>();
    if (!pos_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW"
                " class.", getHardwareInterfaceType().c_str());
      return false;
    }

    // Get a pointer to the force-torque sensor interface
    // ForceTorqueSensorInterface *ft_iface = robot_hw->get<ForceTorqueSensorInterface>();

    // if (!ft_iface)
    // {
    //   ROS_ERROR("This controller requires a hardware interface of type '%s'."
    //             " Make sure this is registered in the hardware_interface::RobotHW"
    //             " class.",
    //             internal::demangledTypeName<ForceTorqueSensorInterface>().c_str());
    //   return false;
    // }

    ////////////////////////////////////////////////////////////////////////////
    // load handles
    ////////////////////////////////////////////////////////////////////////////

    // Load Hardware interfaces
    if (!initJoints(pos_iface, controller_nh) /*!||
        initForceTorqueSensors(ft_iface, controller_nh)*/)
    {
      ROS_ERROR_STREAM("TOMMControlPluginBase::init(): Failed to initialize harware interface '"
        << hardware_interface::internal::demangledTypeName(*this) << "'");
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Setup internal controller
    ////////////////////////////////////////////////////////////////////////////

    // initalize the everything
    if (!internalInit(root_nh, controller_nh, claimed_resources))
    {
      ROS_ERROR("Failed to initialize the controller");
      std::cerr << "FAILED LOADING PLUGIN" << std::endl;
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Claim the resources
    ////////////////////////////////////////////////////////////////////////////
    if(claim_resources_)
    {
      pos_iface->clearClaims();
      claimed_resources.push_back(
          InterfaceResources(
              internal::demangledTypeName<PositionJointInterface>(),
              pos_iface->getClaims()));
      pos_iface->clearClaims();
    }

    // success
    state_ = ControllerBase::ControllerState::INITIALIZED;
    return true;
  }

  void TOMMControlPluginBase::starting(const ros::Time &time)
  {
    internalStarting(time);
  }

  void TOMMControlPluginBase::update(const ros::Time &time, const ros::Duration &period)
  {
    internalUpdate(time, period);
    ros::spinOnce();
  }

  void TOMMControlPluginBase::stopping(const ros::Time &time)
  {
    internalStopping(time);
  }

  bool TOMMControlPluginBase::initJoints(hardware_interface::PosVelJointInterface *pos_iface, ros::NodeHandle &nh)
  {
    // Get joint names from the parameter server
    using namespace XmlRpc;

    XmlRpcValue joint_names;

    if (!nh.getParam("joints", joint_names))
    {
      ROS_ERROR_STREAM("No joints given (namespace:" << nh.getNamespace() << ").");
      return false;
    }

    if (joint_names.getType() != XmlRpcValue::TypeArray)
    {
      ROS_ERROR_STREAM("Malformed joint specification (namespace:" << nh.getNamespace() << ").");
      return false;
    }

    // Populate container of joint handles
    for (int i = 0; i < joint_names.size(); ++i)
    {
      XmlRpcValue &name_value = joint_names[i];

      if (name_value.getType() != XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM("Array of joint names should contain all strings"
                         " (namespace:"
                         << nh.getNamespace() << ").");
        return false;
      }
      const std::string joint_name = static_cast<std::string>(name_value);

      // Get a joint handle
      try
      {
        joint_handles_.push_back(
          std::make_shared<JointHandle>(pos_iface->getHandle(joint_name)));
        ROS_DEBUG_STREAM("Found joint '" << joint_name << "' in '"
                                         << getHardwareInterfaceType() << "'");
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find joint '" << joint_name << "' in '"
                                                  << getHardwareInterfaceType() << "'");
        return false;
      }
    }

    ROS_INFO("TOMMControlPluginBase: got %ld joint handles", joint_handles_.size());
    return true;
  }

  bool TOMMControlPluginBase::initForceTorqueSensors(
    hardware_interface::ForceTorqueSensorInterface *ft_iface, ros::NodeHandle &nh)
  {
    // Get ft_sensor names from the parameter server
    using namespace XmlRpc;
    XmlRpcValue ft_sensor_names;
    if (!nh.getParam("ft_sensors", ft_sensor_names))
    {
      ROS_ERROR_STREAM("No ft_sensors given (namespace:" << nh.getNamespace() << ").");
      return false;
    }
    if (ft_sensor_names.getType() != XmlRpcValue::TypeArray)
    {
      ROS_ERROR_STREAM("Malformed ft_sensor specification (namespace:" << nh.getNamespace() << ").");
      return false;
    }

    // Populate container of force torque sensors
    for (int i = 0; i < ft_sensor_names.size(); ++i)
    {
      XmlRpcValue &name_value = ft_sensor_names[i];

      if (name_value.getType() != XmlRpcValue::TypeString)
      {
        ROS_ERROR_STREAM("Array of ft_sensor names should contain all strings"
                         " (namespace:" << nh.getNamespace() << ").");
        return false;
      }
      const std::string ft_sensor_name = static_cast<std::string>(name_value);

      // Get a joint handle
      try
      {
        ft_handles_.push_back(std::make_shared<FTHandle>(
          ft_iface->getHandle(ft_sensor_name)));
        ROS_DEBUG_STREAM("Found ft_sensor '" << ft_sensor_name << "' in '"
                                             << getHardwareInterfaceType() << "'");
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find ft_sensor '" << ft_sensor_name
                                                      << "' in '" << getHardwareInterfaceType() << "'");
        return false;
      }
    }

    ROS_INFO("TOMMControlPluginBase: got %ld ft handles", ft_handles_.size());
    return true;
  }

} // namespace tomm_control_plugin
