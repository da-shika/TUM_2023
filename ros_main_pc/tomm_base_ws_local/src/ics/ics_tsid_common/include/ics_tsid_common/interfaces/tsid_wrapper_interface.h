#ifndef ICS_TSID_COMMON_TSID_INTERFACE_H_
#define ICS_TSID_COMMON_TSID_INTERFACE_H_

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/interfaces/robot_wrapper_interface.h>
#include <ics_tsid_common/utilities/tasks.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types.h>

namespace ics
{

  /**
   * @brief Interface Class for TSIDWrapper
   * 
   */
  class TSIDWrapperInterface
  {
    public:
      typedef tsid::InverseDynamicsFormulationAccForce Formulation;
      typedef std::shared_ptr<tsid::tasks::TaskBase> TaskPtr;
      typedef std::shared_ptr<tsid::contacts::Contact6dExt> ContactPtr;
      typedef tsid::solvers::SolverHQPBase Solver;
      typedef Formulation::Data Data;

    public:

      /**
       * @brief check if task in the formulation
       */
      virtual  bool hasTask(const std::string &name) const = 0;

      /**
       * @brief check if contact in the formulation
       */
      virtual bool hasContact(const std::string &name) const = 0;

      /**
       * @brief load
       */
      virtual bool loadTask(const std::string& name, TaskPtr task, cc::Scalar weight = -1, unsigned int priority = 1, bool activate=true) = 0;

      /**
       * @brief load/add a contact 
       */
      virtual bool loadContact(const std::string &name, ContactPtr contact, cc::Scalar transition_time = 0, bool activate=true) = 0;

      /**
       * @brief add task
       */
      virtual bool addTask(const std::string &name, cc::Scalar weight = -1, unsigned int priority = 1) = 0;

      /**
       * @brief remove task
       */
      virtual bool removeTask(const std::string &name, cc::Scalar transition_time = 0) = 0;

      /**
       * @brief add contact
       */
      virtual bool addContact(const std::string &name, cc::Scalar transition_time = 0) = 0;

      /**
       * @brief remove contact
       */
      virtual bool removeContact(const std::string &name, cc::Scalar transition_time = 0) = 0;

      /**
       * @brief access to robot configuration
       * 
       * @return const cc::VectorX& 
       */
      virtual const cc::VectorX &q() const = 0;

      /**
       * @brief access to generalized robot velocity
       * 
       * @return const cc::VectorX& 
       */
      virtual const cc::VectorX &v() const = 0;

      /**
       * @brief access to generalized robot accelerations
       * 
       * @return const cc::VectorX& 
       */
      virtual const cc::VectorX &a() const = 0;

      /**
       * @brief access to the tsid formulation
       * 
       * @return tsid::InverseDynamicsFormulationAccForce& 
       */
      virtual Formulation &formulation() = 0;
      virtual const Formulation &formulation() const = 0;

      /**
       * @brief access to the tsid solver
       * 
       * @return Solver& 
       */
      virtual Solver &solver() = 0;
      virtual const Solver &solver() const = 0;
      virtual const std::int64_t &solverDuration() const = 0;

      /**
       * @brief access to the robot Interface used by tsid
       * 
       * @return RobotInterface& 
       */
      virtual RobotWrapperInterface &robotInterface() = 0;
      virtual const RobotWrapperInterface &robotInterface() const = 0;

      /**
       * @brief access to the data
       * 
       * @return tsid::InverseDynamicsFormulationAccForce& 
       */
      virtual Data &data() = 0;
      virtual const Data &data() const = 0;

      /**
       * @brief access to geometry data
       * 
       * note: this added here since tasks use the commanded robot to
       * compute collision avoidance tasks.
       * 
       * @return pinocchio::GeometryModel& 
       */
      virtual pinocchio::GeometryData& geometryData() = 0;
      virtual const pinocchio::GeometryData& geometryData() const = 0;
      virtual const std::int64_t &geometryDuration() const = 0;

      /**
       * @brief print status
       */
      virtual std::string toString() const = 0;
  };

}

#endif