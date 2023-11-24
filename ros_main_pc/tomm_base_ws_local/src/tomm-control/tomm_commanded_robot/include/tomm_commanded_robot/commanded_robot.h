#ifndef TOMM_COMMANDED_ROBOT_COMMANDED_ROBOT_H_
#define TOMM_COMMANDED_ROBOT_COMMANDED_ROBOT_H_

////////////////////////////////////////////////////////////////////////////////
// Using Pinocchio with Fast collision lib
////////////////////////////////////////////////////////////////////////////////
#define PINOCCHIO_WITH_HPP_FCL

#include <tomm_core/mobile_manipulator_formulation.h>

#include <tomm_interfaces/mobile_manipulator_interface.h>

#include <control_core/interfaces/module_base.h>

namespace tomm
{

  /**
   * @brief CommandedRobot Class
   *
   * Holds the commanded robot states computed by the formulation.
   * That is, the virtual state of where TSID things the robot should be.
   *
   * Modules:
   *  - RobotWrapper: pinocchio
   *  - HumanoidFormulation: TSID stack for humanoid robot
   */
  class CommandedRobot : public cc::ModuleBase,
                         public MobileManipulatorInterface
  {
  public:
    typedef cc::ModuleBase Base;
    typedef MobileManipulatorInterface Interface;

  private:
    //////////////////////////////////////////////////////////////////////////
    // states
    //////////////////////////////////////////////////////////////////////////
    cc::RobotState state_;
    // std::vector<cc::LinearPosition> contact_zmp_;

    //////////////////////////////////////////////////////////////////////////
    // modules
    //////////////////////////////////////////////////////////////////////////
    std::unique_ptr<ics::RobotWrapper> robot_;
    std::unique_ptr<tomm::MobileManipulatorFormulation> formulation_;

  public:
    CommandedRobot(const std::string &name);
    virtual ~CommandedRobot();

    //////////////////////////////////////////////////////////////////////////
    // program flow
    //////////////////////////////////////////////////////////////////////////

    /**
     * @brief set start joint and fb state
     *
     * Note: This computes the fb state by projecting the robot on the ground.
     * This sets all references to match current state to avoid jumps.
     *
     */
    void start(
        const cc::JointState &joint_state,
        const ros::Time &time);

    /**
     * @brief set (overrides!) internal integrator state
     */
    void setState(
        const cc::JointState &joint_state, bool recompute = false);

    /**
     * @brief update the idyn problem based on joint and fb state
     *
     * Computes generalized accelerations based on state (joint_state,fb_state).
     * Internally integrates solution in (q_virtual, v_virtual)
     */
    bool update(
        const cc::JointState &joint_state,
        const ros::Time &time,
        const ros::Duration &period);

    //////////////////////////////////////////////////////////////////////////
    // interface
    //////////////////////////////////////////////////////////////////////////

    virtual cc::Scalar mass() const override
    {
      return robot_->totalMass();
    }

    virtual std::vector<std::string> actuatedJointNames() const
    {
      return robot_->actuatedJointNames();
    }

    virtual const cc::VectorX &q() const override
    {
      return formulation_->idyn().q();
    }
    virtual const cc::VectorX &v() const override
    {
      return formulation_->idyn().v();
    }
    const cc::VectorX &a() const
    {
      return formulation_->idyn().a();
    }

    virtual const cc::JointState &jointState() const override
    {
      return state_.joints();
    }

    virtual const cc::CartesianState &baseState() const override
    {
      return formulation_->baseState();
    }

    virtual const cc::RobotState &robotState() const
    {
      return state_;
    }

    virtual const cc::CartesianState &bodyState(const cc::BodyId &id) const override
    {
      return state_.body(id);
    }

    // virtual const cc::LinearPosition &contactZmp(const cc::BodyId &id) const override
    // {
    //   return contact_zmp_[id];
    // }

    virtual const cc::Wrench &contactWrench(const cc::BodyId &id) const override
    {
      return state_.ftSensor(id).wrench();
    }

    const MobileManipulatorFormulation &formulation() const
    {
      return *formulation_;
    }
    MobileManipulatorFormulation &formulation()
    {
      return *formulation_;
    }

    virtual std::string toString() const override;

  protected:
    virtual bool init(ros::NodeHandle &nh, Base::Parameters &global_params) override;

  private:
    void updateStates();
  };

}

#endif