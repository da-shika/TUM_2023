#ifndef TOMM_CORE_MOBILE_MANIPULATOR_FORMULATION_H_
#define TOMM_CORE_MOBILE_MANIPULATOR_FORMULATION_H_

////////////////////////////////////////////////////////////////////////////////
// Using Pinocchio with Fast collision lib
////////////////////////////////////////////////////////////////////////////////
#define PINOCCHIO_WITH_HPP_FCL

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <ics_tsid_tasks/tsid/task_joint_pos_vel_acc_bounds_mobile.h>
#include <ics_tsid_tasks/tsid/task_joint_posture_mobile.h>

////////////////////////////////////////////////////////////////////////////////
// ics_formulation incldues
////////////////////////////////////////////////////////////////////////////////
#include <ics_formulation/formulation_interface.h>

#include <control_core/interfaces/module_base.h>

namespace tomm
{

  class MobileManipulatorFormulation : public cc::ModuleBase,
                                       public ics::FormulationInterface
  {
  public:
    typedef ModuleBase Base;
    typedef std::shared_ptr<ics::BodyPartFormulation> BodyPartPtr;
    typedef std::vector<BodyPartPtr> BodyParts;
    typedef tsid::trajectories::TrajectorySample TrajectorySample;

    typedef std::shared_ptr<tsid::tasks::TaskSE3Equality> BaseTask;
    typedef std::shared_ptr<tsid::tasks::TaskJointPosture> PostureTask;         // TODO: Hardcoded!!!
    typedef std::shared_ptr<tsid::tasks::TaskJointPosVelAccBounds> BoundsTask;  // TODO: Hardcoded!!!

  private:
    //////////////////////////////////////////////////////////////////////////
    // modules
    //////////////////////////////////////////////////////////////////////////
    ics::TSIDWrapper idyn_;
    BodyParts bodies_;

    //////////////////////////////////////////////////////////////////////////
    // humanoid default tasks
    //////////////////////////////////////////////////////////////////////////
    std::string base_name_;
    std::string posture_name_;
    std::string bounds_name_;

    BaseTask base_task_;
    PostureTask posture_task_;
    BoundsTask bounds_task_;

    //////////////////////////////////////////////////////////////////////////
    // task states
    //////////////////////////////////////////////////////////////////////////
    cc::CartesianState base_state_;
    cc::JointState joint_state_;

    //////////////////////////////////////////////////////////////////////////
    // inputs
    //////////////////////////////////////////////////////////////////////////
    cc::VectorX q_in_;
    cc::VectorX v_in_;

  public:
    MobileManipulatorFormulation(
        ics::RobotWrapper &robot,
        const std::string &name);
    virtual ~MobileManipulatorFormulation();

    //////////////////////////////////////////////////////////////////////////
    // set states
    //////////////////////////////////////////////////////////////////////////

    /**
     * @brief set the joint and fb state
     *
     * note: this recomputes the problem data if true.
     * Otherwise just overrides internal robot state
     */
    void setState(const cc::JointState &joint_state, bool recompute = false);

    //////////////////////////////////////////////////////////////////////////
    // update formulation
    //////////////////////////////////////////////////////////////////////////

    /**
     * @brief solves the idyn formulation using joint and fb state as input
     *
     * note: this also recomputes the problem data
     */
    bool update(
        const cc::JointState &joint_state,
        const ros::Time &time,
        const ros::Duration &period);

    /**
     * @brief convert states to string
     */
    std::string toString() const;

    //////////////////////////////////////////////////////////////////////////
    // get/add/remove tasks
    //////////////////////////////////////////////////////////////////////////

    bool addBaseTask();
    bool addPostureTask();
    bool addBoundsTask();

    BaseTask baseTask() { return base_task_; }
    PostureTask postureTask() { return posture_task_; }
    BoundsTask boundsTask() { return bounds_task_; }

    bool removeBaseTask(cc::Scalar transition_time = 0);
    bool removePostureTask(cc::Scalar transition_time = 0);
    bool removeBoundsTask(cc::Scalar transition_time = 0);

    //////////////////////////////////////////////////////////////////////////
    // set references
    //////////////////////////////////////////////////////////////////////////

    void setBaseReference(TrajectorySample &ref);
    void setBaseReference(const cc::CartesianState &ref);
    void setPostureReference(TrajectorySample &ref);
    virtual void setPostureReference(const cc::JointState &ref) override;

    //////////////////////////////////////////////////////////////////////////
    // get states
    //////////////////////////////////////////////////////////////////////////

    const cc::CartesianState &baseState();
    const cc::JointState &jointState();

    virtual const cc::JointState &postureState() const { return joint_state_; }

    virtual const cc::CartesianPosition &basePosition() const override { return base_state_.pos(); }
    virtual const cc::CartesianState &baseState() const override { return base_state_; }

    virtual cc::CartesianPosition framePosition(const std::string &name) const override;
    virtual cc::CartesianPosition framePosition(pinocchio::FrameIndex id) const override;

    virtual cc::CartesianState frameState(const std::string &name) const override;
    virtual cc::CartesianState frameState(pinocchio::FrameIndex id) const override;

    ////////////////////////////////////////////////////////////////////////////
    // body parts
    ////////////////////////////////////////////////////////////////////////////

    virtual const std::vector<BodyPartPtr> bodies() const override { return bodies_; }
    virtual std::vector<BodyPartPtr> bodies() override { return bodies_; }

    virtual const BodyPartPtr body(const cc::BodyId &id) const override { return bodies_[id]; }
    virtual BodyPartPtr body(const cc::BodyId &id) override { return bodies_[id]; }

    ////////////////////////////////////////////////////////////////////////////
    // inverse dynamic formulation
    ////////////////////////////////////////////////////////////////////////////

    virtual ics::TSIDWrapper &idyn() override { return idyn_; }
    virtual const ics::TSIDWrapper &idyn() const override { return idyn_; }

    ////////////////////////////////////////////////////////////////////////////
    // load / unload task
    ////////////////////////////////////////////////////////////////////////////

    virtual bool loadTasksFromParameters(                          
      ros::NodeHandle& nh, const std::vector<std::string> &names, bool activate, bool verbose);

    virtual bool unLoadTasks(
      const std::vector<std::string> &names, bool verbose);

  protected:
    virtual bool init(ros::NodeHandle &nh, Base::Parameters &global_params) override;

    void q_to_tsid(const cc::JointPosition& q, cc::VectorX& q_tsid);
    void q_from_tsid(const cc::VectorX& q_tsid, cc::JointPosition& q);
  };

}

#endif