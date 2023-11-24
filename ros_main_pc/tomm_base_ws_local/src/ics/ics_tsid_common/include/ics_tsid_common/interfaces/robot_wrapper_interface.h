#ifndef ICS_TSID_COMMON_ROBOT_INTERFACE_H_
#define ICS_TSID_COMMON_ROBOT_INTERFACE_H_

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/robots/robot-wrapper.hpp>

namespace ics
{

  /**
   * @brief Interface Class for RobotWrapper
   * 
   */
  class RobotWrapperInterface
  {
    public:
      typedef pinocchio::Data Data;

    public:
      /**
       * @brief access to the tsid robot wrapper
       * 
       * @return tsid::robots::RobotWrapper& 
       */
      virtual tsid::robots::RobotWrapper& wrapper() = 0;
      virtual const tsid::robots::RobotWrapper& wrapper() const = 0;

      /**
       * @brief access to the pinocchio model
       * 
       * @return pinocchio::Model& 
       */
      virtual pinocchio::Model& model() = 0;
      virtual const pinocchio::Model& model() const = 0;

      /**
       * @brief access to the hpp-fcl geometry model
       * 
       * @return pinocchio::GeometryModel& 
       */
      virtual pinocchio::GeometryModel& geometryModel() = 0;
      virtual const pinocchio::GeometryModel& geometryModel() const = 0;

      /**
       * @brief print status
       */
      virtual std::string toString() const = 0;
  };

}

#endif