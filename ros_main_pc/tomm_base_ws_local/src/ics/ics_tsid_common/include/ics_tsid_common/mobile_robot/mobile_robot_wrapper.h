#ifndef ICS_TSID_COMMON_mobile_robot_wrapper_hpp__
#define ICS_TSID_COMMON_mobile_robot_wrapper_hpp__

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/robots/robot-wrapper.hpp>

namespace tsid
{
  namespace robots
  {
    class MobileRobotWrapper : public RobotWrapper
    {
    public:
      MobileRobotWrapper(const pinocchio::Model &m, bool verbose = false);
    };
  }
}

#endif