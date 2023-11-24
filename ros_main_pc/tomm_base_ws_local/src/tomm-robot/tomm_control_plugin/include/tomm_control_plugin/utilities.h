#ifndef TOMM_CONTROL_PLUGIN_UTILITIES_H_
#define TOMM_CONTROL_PLUGIN_UTILITIES_H_

////////////////////////////////////////////////////////////////////////////////
// control_core inclues
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types.h>

///////////////////////////////////////////////////////////////////////////////
// ros_control includes
////////////////////////////////////////////////////////////////////////////////
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace tomm_control_plugin
{

  typedef hardware_interface::ForceTorqueSensorHandle FTHandle;
  typedef hardware_interface::PosVelJointHandle JointHandle;
  typedef std::vector<std::shared_ptr<FTHandle> > FTHandles;
  typedef std::vector<std::shared_ptr<JointHandle> > JointHandles;

  /**
   * @brief read handlers into cc::RobotState
   * 
   * @param joint_handles >
   * @param imu_handle 
   * @param ft_handles 
   * @param state 
   */
  inline void read(
    const JointHandles& joint_handles,
    const FTHandles& ft_handles,
    cc::RobotState& state)
  {
    // joint state
    auto& joints = state.joints();

    for(size_t i = 0; i < joint_handles.size(); ++i)
    {
      joints.pos()[i] = joint_handles[i]->getPosition();
      joints.vel()[i] = joint_handles[i]->getVelocity();
    }

    // ft sensors
    // for(size_t i = 0; i < ft_handles.size(); ++i)
    // {
    //   const auto& ft_handle = ft_handles[i];
    //   auto& ft = state.ftSensors()[i];
    //   ft.W().force().x() = ft_handle->getForce()[0];
    //   ft.W().force().y() = ft_handle->getForce()[1];
    //   ft.W().force().z() = ft_handle->getForce()[2];
    //   ft.W().moment().x() = ft_handle->getTorque()[0];
    //   ft.W().moment().y() = ft_handle->getTorque()[1];
    //   ft.W().moment().z() = ft_handle->getTorque()[2];
    // }
  }

  /**
   * @brief write command to JointHandles
   * 
   * @param cmd 
   * @param joint_handles 
   * @return true 
   * @return false 
   */
  inline bool write(const cc::JointState& cmd, JointHandles& joint_handles)
  {
    auto& q = cmd.pos();
    auto& v = cmd.vel();
    for(size_t i = 0; i < joint_handles.size(); ++i)
    {
      if(std::isnan(q[i]))
        return false;
      joint_handles[i]->setCommandPosition(q[i]);
      joint_handles[i]->setCommandVelocity(v[i]);
    }
    return true;
  }

  /**
   * @brief print the current hardware readings
   * 
   * @param joint_handles 
   * @param ft_handles 
   * @return std::string 
   */
  inline std::string print_handles(
    const JointHandles& joint_handles,
    const FTHandles& ft_handles)
  {  
    std::stringstream ss;

    // joint state
    ss << "joint_handles.pos=[";
    for(size_t i = 0; i < joint_handles.size(); ++i)
      ss << joint_handles[i]->getPosition() << ", ";
    ss << "]" << std::endl;

    ss << "joint_handles.vel=[";
    for(size_t i = 0; i < joint_handles.size(); ++i)
      ss << joint_handles[i]->getVelocity() << ", ";
    ss << "]" << std::endl;

    // ft sensors
    for(size_t i = 0; i < ft_handles.size(); ++i)
    {
      const auto& ft_handle = ft_handles[i];

      ss << "ft_handle[" << i << "].W=[" << 
        ft_handle->getForce()[0] << ", " << 
        ft_handle->getForce()[1] << ", " << 
        ft_handle->getForce()[2] << ", " << 
        ft_handle->getTorque()[0] << ", " << 
        ft_handle->getTorque()[1] << ", " << 
        ft_handle->getTorque()[2] << "]" << std::endl;
    }
    return ss.str();
  }

}

#endif
