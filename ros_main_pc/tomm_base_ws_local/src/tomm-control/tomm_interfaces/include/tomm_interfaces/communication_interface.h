#ifndef TOMM_INTERFACES_COMMUNICATION_INTERFACE_H_
#define TOMM_INTERFACES_COMMUNICATION_INTERFACE_H_

#include <control_core/types.h>

namespace tomm
{

  /**
   * @brief CommmunicationInterface Class
   * 
   * Communication with real/simulated robot
   */
  class CommunicationInterface
  {
    public:
      /**
       * @brief check if connection
       */
      virtual bool isConnected() const = 0;

      /**
       * @brief access to robot state 
       */
      virtual const cc::RobotState& robotState() const = 0;
      virtual cc::RobotState& robotState() = 0;

      /**
       * @brief access to command
       */
      virtual const cc::JointState& command() const = 0;
      virtual cc::JointState& command() = 0;
  };

}

#endif