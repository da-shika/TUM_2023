#include <ics_formulation/body_part_formulation.h>

#include <ics_tsid_tasks/tasks/task_factory.h>

namespace ics
{

  BodyPartFormulation::BodyPartFormulation(ics::TSIDWrapper& tsid, const std::string& name) : 
    Base(name),
    tsid_(tsid),
    state_(FREE),
    motion_task_(nullptr),
    contact_task_(nullptr)
  {
  }

  BodyPartFormulation::~BodyPartFormulation()
  { 
  }

  bool BodyPartFormulation::init(ros::NodeHandle &nh, Base::Parameters &global_params)
  {
    ////////////////////////////////////////////////////////////////////////////
    // load parametes
    ////////////////////////////////////////////////////////////////////////////
    cc::Parameters parameter(nh, Base::name());
    parameter.addRequired<std::string>("motion");
    parameter.addOptional<std::string>("contact", "");
    if(!parameter.load())
    {
      PRINT_ERROR("Can't load parameter in ns '%s'", parameter.privateNamespace().c_str());
      return false;
    }
    parameter.get("motion", motion_name_);
    parameter.get("contact", contact_name_);

    ////////////////////////////////////////////////////////////////////////////
    // add them to the formulation and extract ptrs
    ////////////////////////////////////////////////////////////////////////////
    std::vector<std::string> tasks;
    if(parameter.isLoaded("contact"))
    {
      // we have a contact task
      if(!ics::TaskFactory::LoadList(tsid_, nh, {motion_name_, contact_name_}, true, true))
      {
        PRINT_ERROR("Can't load tasks '%s', '%s'", motion_name_.c_str(), contact_name_.c_str());
        return false;
      }
      motion_task_ = tsid_.task<tsid::tasks::TaskSE3Equality>(motion_name_);
      contact_task_ = tsid_.contactTask(contact_name_);
    }
    else
    {
      // we have no contact task
      if(!ics::TaskFactory::LoadList(tsid_, nh, {motion_name_}, true, true))
      {
        PRINT_ERROR("Can't load tasks '%s'", motion_name_.c_str());
        return false;
      }
      motion_task_ = tsid_.task<tsid::tasks::TaskSE3Equality>(motion_name_);
    }

    ////////////////////////////////////////////////////////////////////////////
    // deactivate all tasks per default
    ////////////////////////////////////////////////////////////////////////////
    tsid_.removeTask(motion_name_);
    if(contact_task_)
      tsid_.removeContact(contact_name_);
    state_ = FREE;

    return true;
  }

  //////////////////////////////////////////////////////////////////////////
  // motion task
  //////////////////////////////////////////////////////////////////////////

  bool BodyPartFormulation::addMotion()
  {
    if(state_ == CONTACT)
    {
      PRINT_WARN("Contact Task still active");
      return false;
    }
    if(!tsid_.addTask(motion_name_))
      return false;
    state_ = MOTION;
    return true;
  }

  bool BodyPartFormulation::removeMotion()
  {
    if(state_ != MOTION)
    {
      PRINT_WARN("Motion Task not active");
      return false;
    }
    if(!tsid_.removeTask(motion_name_))
      return false;
    state_ = FREE;
    return true;
  }

  void BodyPartFormulation::setMotionReference(TrajectorySample &ref)
  {
    motion_task_->setReference(ref);
    if(contact_task_)
      contact_task_->setMotionReference(ref);                                   
  }
  void BodyPartFormulation::setMotionReference(const cc::CartesianPosition &ref)
  {
    auto sample = ics::to_sample(ref);
    setMotionReference(sample);
  }
  void BodyPartFormulation::setMotionReference(const cc::CartesianState &ref)
  {
    auto sample = ics::to_sample(ref);
    motion_task_->setReference(sample);
    if(contact_task_)
      contact_task_->setMotionReference(sample);
  }

  cc::CartesianState BodyPartFormulation::motionReference()
  {
    return ics::to_cartesian_state(motion_task_->getReference());
  }

  cc::CartesianPosition BodyPartFormulation::motionPosition()
  {
    return ics::to_pose(
      tsid_.robot().framePosition(tsid_.data(), motion_task_->frame_id()));
  }

  cc::CartesianState BodyPartFormulation::motionState()
  {
    return ics::to_cartesian_state(
      tsid_.robot().frameState(tsid_.data(), motion_task_->frame_id()));
  }

  bool BodyPartFormulation::updateMotionWeight(cc::Scalar weight)
  {
    return tsid_.updateTaskWeight(motion_name_, weight);
  }

  bool BodyPartFormulation::updateTaskGains(const cc::VectorX &Kp, const cc::VectorX &Kd)
  {
    return tsid_.updateTaskGains<tsid::tasks::TaskSE3Equality>(motion_name_, Kp, Kd);
  }

  void BodyPartFormulation::udpateMotionMask(const cc::VectorX &mask)
  {
    tsid_.udpateTaskMask(motion_name_, mask);
  }

  //////////////////////////////////////////////////////////////////////////
  // contact task
  //////////////////////////////////////////////////////////////////////////

  bool BodyPartFormulation::addContact(const cc::CartesianPosition &ref, cc::Scalar transition_time)
  {
    PRINT_ASSERT(contact_task_, "No contact loaded");

    if(state_ == MOTION)
    {
      PRINT_WARN("Motion Task still active");
      return false;
    }
    auto se3 = ics::to_se3(ref);
    if(se3.isIdentity())
      contact_task_->setReference(ics::to_se3(motionPosition()));               // TODO set to reference instead?
    else
      contact_task_->setReference(se3);                                         
    if(!tsid_.addContact(contact_name_, transition_time))                       // TODO: set force reference to zero?
      return false;
    state_ = CONTACT;
    return true;
  }

  bool BodyPartFormulation::removeContact(cc::Scalar transition_time)
  {
    PRINT_ASSERT(contact_task_, "No contact loaded");

    if(state_ != CONTACT)
    {
      PRINT_WARN("Contact Task not active");
      return false;
    }
    if(!tsid_.removeContact(contact_name_, transition_time))
      return false;
    state_ = FREE;
    return true;
  }

  bool BodyPartFormulation::setForceReference(const cc::Wrench& wrench)
  {
    PRINT_ASSERT(contact_task_, "No contact loaded");
    
    if(state_ != CONTACT)
      return false;
    contact_task_->setForceReference(wrench);
    return true;
  }

  bool BodyPartFormulation::updateRigidContactWeights(cc::Scalar force_regularization_weight, cc::Scalar motion_weight)
  {
    return tsid_.updateRigidContactWeights(contact_name_, force_regularization_weight, motion_weight);
  }

  bool BodyPartFormulation::updateContactGains(const cc::Vector6 &Kp, const cc::VectorX &Kd)
  {
    return tsid_.updateContactGains(contact_name_, Kp, Kd);
  }

  cc::Wrench BodyPartFormulation::contactWrench()
  {
    return tsid_.contactWrenchSolution(contact_name_);
  }

  cc::Scalar BodyPartFormulation::contactNormalForce()
  {
    return tsid_.contactNormalForceSolution(contact_name_);
  }

}
