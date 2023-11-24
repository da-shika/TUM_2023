#include <ics_tsid_common/mobile_robot/mobile_robot_wrapper.h>

namespace tsid
{
  namespace robots
  {
    MobileRobotWrapper::MobileRobotWrapper(const pinocchio::Model &m, bool verbose)
        : RobotWrapper(m, verbose)
    {
      m_model = m;
      m_model_filename = "";
      m_na = m_model.nv;
      init();
    }
  }
}