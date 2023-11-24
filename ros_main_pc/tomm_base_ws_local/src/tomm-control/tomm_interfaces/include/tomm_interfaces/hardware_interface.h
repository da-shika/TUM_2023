#ifndef TOMM_INTERFACES_HARDWARE_INTERFACE_H_
#define TOMM_INTERFACES_HARDWARE_INTERFACE_H_

#include <control_core/types.h>

namespace tomm
{

  /**
   * @brief HardwareInterface Class
   * 
   * Used by the ros control plugin to read robot state and write 
   * joint commands.
   * 
   */
  class HardwareInterface
  {
    public:
    
      //////////////////////////////////////////////////////////////////////////
      // program flow functions
      //////////////////////////////////////////////////////////////////////////

      virtual bool sendCommand(const cc::JointState& cmd) = 0;

      //////////////////////////////////////////////////////////////////////////
      // hardware states
      //////////////////////////////////////////////////////////////////////////

      virtual const cc::JointState& jointState() const = 0;

      virtual const std::vector<cc::FtSensor>& ftSensors() const = 0;

      virtual const cc::JointState& commandedJointState() const = 0;

      virtual const cc::JointState& lastcommandedJointState() const = 0;

      virtual std::string toString() const { return std::string(); }
  };

}

#endif