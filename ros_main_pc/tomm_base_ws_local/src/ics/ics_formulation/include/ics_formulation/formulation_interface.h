#ifndef ICS_INTERFACES_FORMULATION_INTERFACE_H_
#define ICS_INTERFACES_FORMULATION_INTERFACE_H_

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_wrapper/tsid_wrapper.h>
#include <ics_robot_wrapper/robot_wrapper.h>
#include <ics_formulation/body_part_formulation.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types.h>

namespace ics
{
  /**
   * @brief FormulationInterface
   *
   * Interface for formulations. Contains access function that should be
   * the same for every robot formulation. If behaviors are formulated
   * against this interface we can make the multi-robot.
   * 
   * Minimum setup would be:
   *  - posture task
   * 
   */
  class FormulationInterface
  {
  public:
    typedef std::shared_ptr<BodyPartFormulation> BodyPartPtr;
    typedef std::vector<BodyPartPtr> BodyParts;

    ////////////////////////////////////////////////////////////////////////////
    // position / state access
    ////////////////////////////////////////////////////////////////////////////

    /**
     * @brief set the posture
     */
    virtual void setPostureReference(const cc::JointState &ref) = 0;

    /**
     * @brief get robot posture
     */
    virtual const cc::JointState &postureState() const = 0;

    /**
     * @brief get base position
     */
    virtual const cc::CartesianPosition &basePosition() const = 0;

    /**
     * @brief get base state
     */
    virtual const cc::CartesianState &baseState() const = 0;

    /**
     * @brief get frame position
     */
    virtual cc::CartesianPosition framePosition(const std::string &name) const = 0;
    virtual cc::CartesianPosition framePosition(pinocchio::FrameIndex id) const = 0;

    /**
     * @brief get frame state
     */
    virtual cc::CartesianState frameState(const std::string &name) const = 0;
    virtual cc::CartesianState frameState(pinocchio::FrameIndex id) const = 0;

    ////////////////////////////////////////////////////////////////////////////
    // limb access
    ////////////////////////////////////////////////////////////////////////////

    /**
     * @brief access to body parts
     */
    virtual const std::vector<BodyPartPtr> bodies() const = 0;
    virtual std::vector<BodyPartPtr> bodies() = 0;

    /**
     * @brief access to body partes
     */
    virtual const BodyPartPtr body(const cc::BodyId &id) const = 0;
    virtual BodyPartPtr body(const cc::BodyId &id) = 0;

    /**
     * @brief access to TSIDWrapper
     */
    virtual const ics::TSIDWrapper &idyn() const = 0;
    virtual ics::TSIDWrapper &idyn() = 0;

    ////////////////////////////////////////////////////////////////////////////
    // load / unload task
    ////////////////////////////////////////////////////////////////////////////

    virtual bool loadTasksFromParameters(                          
      ros::NodeHandle& nh, const std::vector<std::string> &names, bool activate, bool verbose) = 0;

    virtual bool unLoadTasks(
      const std::vector<std::string> &names, bool verbose) = 0;
  };

}

#endif