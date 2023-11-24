#ifndef TOMM_INTERFACES_MOBILE_MANIPULATOR_INTERFACE_H_
#define TOMM_INTERFACES_MOBILE_MANIPULATOR_INTERFACE_H_

#include <control_core/types.h>

namespace tomm
{

  /**
   * @brief MobileManipulatorInterface Class
   *
   * Interface for commanded and real robot
   */
  class MobileManipulatorInterface
  {
  public:
    virtual cc::Scalar mass() const = 0;
    virtual std::vector<std::string> actuatedJointNames() const = 0;

    virtual const cc::VectorX &q() const = 0;
    virtual const cc::VectorX &v() const = 0;

    virtual const cc::JointState &jointState() const = 0;
    virtual const cc::CartesianState &baseState() const = 0;
    virtual const cc::RobotState &robotState() const = 0;

    virtual const cc::CartesianState &bodyState(const cc::BodyId &id) const = 0;

    // virtual const cc::LinearPosition &contactZmp(const cc::BodyId &id) const = 0;

    virtual const cc::Wrench &contactWrench(const cc::BodyId &id) const = 0;

    virtual std::string toString() const = 0;
  };

}

#endif