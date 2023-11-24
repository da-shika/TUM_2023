#ifndef ICS_INTERFACES_CONTROLLER_BASE_H_
#define ICS_INTERFACES_CONTROLLER_BASE_H_

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/interfaces/controller_base.h>

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_formulation/formulation_interface.h>

namespace ics
{

  /**
   * @brief ControllerBase Class
   * 
   * BaseClass for tsid / pinocchio based controllers.
   * Overload idyn() and robot() to get access to TSIDWrapper and RobotWrapper
   */
  class ControllerBase : public cc::ControllerBase
  {
  public:
    typedef cc::ControllerBase Base;

  public:
    /*!
     * @brief Constructor.
     */
    ControllerBase(const std::string &name) : Base(name)
    {
    }

    /*!
     * @brief Virtual destructor.
     */
    virtual ~ControllerBase()
    {
    }

    /**
     * @brief access to the controllers formulation
     */
    virtual ics::FormulationInterface& formulation() = 0;
    virtual const ics::FormulationInterface& formulation() const = 0;
  };

}

#endif