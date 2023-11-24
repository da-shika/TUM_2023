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

#ifndef TOMM_CONTROL_PLUGIN_CONTROL_PLUGIN_BASE_H_
#define TOMM_CONTROL_PLUGIN_CONTROL_PLUGIN_BASE_H_

////////////////////////////////////////////////////////////////////////////////
// ros hardware includes
////////////////////////////////////////////////////////////////////////////////
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>

namespace tomm_control_plugin
{
  /**
   * @brief TOMMControlPluginBase Class
   * 
   * This class is loaded by the ros_control framework to interact with the 
   * simulator or real robot. It claims resources and sets up all sensors/
   * actuators on the robot.
   * Derived classes reimplement:
   *  internalStarting(), internalUpdate(), internalStopping()
   * 
   */
  class TOMMControlPluginBase : public controller_interface::ControllerBase
  {
  public:
    typedef controller_interface::ControllerBase Base;
    typedef hardware_interface::ForceTorqueSensorHandle FTHandle;
    typedef hardware_interface::ImuSensorHandle ImuHandle;
    typedef hardware_interface::PosVelJointHandle JointHandle;
    typedef std::shared_ptr<ImuHandle> ImuHandlePtr;
    typedef std::vector<std::shared_ptr<FTHandle> > FTHandles;
    typedef std::vector<std::shared_ptr<JointHandle> > JointHandles;

  private:
    bool claim_resources_;

  protected: 
    std::string name_;

    ////////////////////////////////////////////////////////////////////////////
    // hardware handles
    ////////////////////////////////////////////////////////////////////////////
    FTHandles ft_handles_;
    JointHandles joint_handles_;

  public:
    TOMMControlPluginBase(bool claim_resources);

    virtual ~TOMMControlPluginBase();

    const std::string name() const { return name_; }

    ////////////////////////////////////////////////////////////////////////////
    // control interface
    ////////////////////////////////////////////////////////////////////////////
   
    virtual bool initRequest(hardware_interface::RobotHW *robot_hw,
                     ros::NodeHandle &root_nh,
                     ros::NodeHandle &controller_nh,
                     ClaimedResources &claimed_resources) override;

    virtual void starting(const ros::Time &time) override;

    virtual void update(const ros::Time &time, const ros::Duration &period) override;

    virtual void stopping(const ros::Time &time) override;

    std::string getHardwareInterfaceType() const {
      return hardware_interface::internal::
          demangledTypeName<hardware_interface::PosVelJointInterface>(); }

  protected:

    ////////////////////////////////////////////////////////////////////////////
    // internal functions
    ////////////////////////////////////////////////////////////////////////////

    virtual bool internalInit(
      ros::NodeHandle &root_nh,
      ros::NodeHandle &controller_nh,
      ClaimedResources &claimed_resources) = 0;

    virtual void internalStarting(const ros::Time &time) = 0;

    virtual void internalUpdate(const ros::Time &time, const ros::Duration &period) = 0;

    virtual void internalStopping(const ros::Time &time) = 0;

  private:
    bool initJoints(
      hardware_interface::PosVelJointInterface *pos_iface,
      ros::NodeHandle &controller_nh);

    bool initForceTorqueSensors(
      hardware_interface::ForceTorqueSensorInterface *ft_iface,
      ros::NodeHandle &controller_nh);
  };

}

#endif