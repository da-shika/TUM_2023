#ifndef TOMM_INTERFACES_WHOLE_BODY_CONTROLLER_INTERFACE_H_
#define TOMM_INTERFACES_WHOLE_BODY_CONTROLLER_INTERFACE_H_

#include <tomm_interfaces/mobile_manipulator_interface.h>

namespace tomm
{

  /**
   * @brief WholeBodyControllerInterface Class
   *
   * Access interface for the tomm WholeBodyController to
   * real, cmd
   *
   */
  class WholeBodyControllerInterface
  {
  public:
    /**
     * @brief access to real robot interface
     */
    virtual const MobileManipulatorInterface &cmdRobotInterface() const = 0;

    /**
     * @brief access to commanded robot interface
     */
    // virtual const HumanoidInterface &cmdRobotInterface() const = 0;          // TODO not sure if we need this!
    
  };

}

#endif
