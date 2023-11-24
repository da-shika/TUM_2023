#include <tomm_core/mobile_manipulator_formulation.h>

#include <ics_tsid_tasks/tasks/task_factory.h>

#include <control_core/math.h>

namespace tomm
{
  MobileManipulatorFormulation::MobileManipulatorFormulation(ics::TSIDWrapper::Robot &robot, const std::string &name) :
    Base(name),
    idyn_(robot, name)
  {
  }

  MobileManipulatorFormulation::~MobileManipulatorFormulation()
  {
  }

  bool MobileManipulatorFormulation::init(ros::NodeHandle &nh, Base::Parameters &global_params)
  {
    ////////////////////////////////////////////////////////////////////////////
    // setup the formulation
    ////////////////////////////////////////////////////////////////////////////
    if(!idyn_.initRequest(nh, global_params))
    {
      PRINT_ERROR("Can't initalize tsid formulation");
      return false;
    }
    q_in_ = idyn_.q();
    v_in_ = idyn_.v();

    ////////////////////////////////////////////////////////////////////////////
    // load tasks
    ////////////////////////////////////////////////////////////////////////////
    cc::Parameters parameter(nh, Base::name() + "/tasks");
    parameter.addRequired<std::string>("base");
    parameter.addRequired<std::string>("posture");
    parameter.addRequired<std::string>("bounds");
    parameter.addOptional<std::vector<std::string> >("additional_tasks", {});
    if(!parameter.load())
    {
      PRINT_ERROR("Can't load parameter");
      return false;
    };
    parameter.get("base", base_name_);
    parameter.get("posture", posture_name_);
    parameter.get("bounds", bounds_name_);

    std::vector<std::string> tasks, additional_tasks;
    tasks = {posture_name_};        // base_name_, bounds_name_                 // TODO: there is an error with the bounds !!!
    parameter.get("additional_tasks", additional_tasks);

    ////////////////////////////////////////////////////////////////////////////
    // add them to the formulation and extract ptrs
    ////////////////////////////////////////////////////////////////////////////
    std::copy(additional_tasks.begin(), additional_tasks.end(), std::back_inserter(tasks));
    if(!ics::TaskFactory::LoadList(idyn_, nh, tasks, true))
    {
      PRINT_ERROR("Can't load tasks");
      return false;
    }
    base_task_ = idyn_.task<tsid::tasks::TaskSE3Equality>(base_name_);
    posture_task_ = idyn_.task<tsid::tasks::TaskJointPosture>(posture_name_);
    bounds_task_ = idyn_.task<tsid::tasks::TaskJointPosVelAccBounds>(bounds_name_);

    ////////////////////////////////////////////////////////////////////////////
    // setup robot bodies
    ////////////////////////////////////////////////////////////////////////////
    std::string name;
    for(const auto& body_name : cc::BodyId::HandNames())
    {
      name = Base::name() + "/tasks/" + body_name;
      auto body = std::make_shared<ics::BodyPartFormulation>(idyn_, name);
      if(!body->initRequest(nh, global_params))
      {
        PRINT_ERROR("Error initializing BodyPart '%s'", name.c_str());
        return false;
      }
      bodies_.push_back(body);
    }
    // make the bodies_ vector same size as BodyId by copying ptr
    // these means that left&right foot point to left&right hand
    bodies_.push_back(bodies_[0]);
    bodies_.push_back(bodies_[1]);

    ////////////////////////////////////////////////////////////////////////////
    // resize formulation
    ////////////////////////////////////////////////////////////////////////////
    if(!idyn_.resizeSolver(true))
    {
      PRINT_ERROR("Error resizing solver");
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // setup members
    ////////////////////////////////////////////////////////////////////////////
    base_state_.setZero();
    joint_state_.setZero(idyn_.robot().wrapper().na());
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  // set states
  //////////////////////////////////////////////////////////////////////////////
  
  void MobileManipulatorFormulation::setState(
    const cc::JointState& joint_state, bool recompute)
  {
    q_to_tsid(joint_state.q(), q_in_);
    v_in_ = joint_state.qP();
    idyn_.setState(q_in_, v_in_, recompute);
  }

  bool MobileManipulatorFormulation::update(
    const cc::JointState& joint_state, const ros::Time &time, const ros::Duration &period)
  {
    // convert from robot [x,y,th,q] to tsid [x,y,s,c,q] format
    q_to_tsid(joint_state.q(), q_in_);
    v_in_ = joint_state.qP();
    return idyn_.update(q_in_, v_in_, time, period);
  }

  //////////////////////////////////////////////////////////////////////////
  // add/remove tasks
  //////////////////////////////////////////////////////////////////////////

  bool MobileManipulatorFormulation::addBaseTask()
  {
    return idyn_.addTask(base_name_);
  }
  bool MobileManipulatorFormulation::addPostureTask()
  {
    return idyn_.addTask(posture_name_);
  }
  bool MobileManipulatorFormulation::addBoundsTask()
  {
    return idyn_.addTask(bounds_name_);
  }

  bool MobileManipulatorFormulation::removeBaseTask(cc::Scalar transition_time)
  {
    return idyn_.removeTask(base_name_, transition_time);
  }
  bool MobileManipulatorFormulation::removePostureTask(cc::Scalar transition_time)
  {
    return idyn_.removeTask(posture_name_, transition_time);
  }
  bool MobileManipulatorFormulation::removeBoundsTask(cc::Scalar transition_time)
  {
    return idyn_.removeTask(bounds_name_, transition_time);
  }

  //////////////////////////////////////////////////////////////////////////
  // set references
  //////////////////////////////////////////////////////////////////////////

  void MobileManipulatorFormulation::setBaseReference(TrajectorySample &ref)
  {
    base_task_->setReference(ref);
  } 
  void MobileManipulatorFormulation::setBaseReference(const cc::CartesianState &ref)
  {
    auto sample = ics::to_sample(ref);
    setBaseReference(sample);
  }
  void MobileManipulatorFormulation::setPostureReference(TrajectorySample &ref)
  {
    posture_task_->setReference(ref);
  }
  void MobileManipulatorFormulation::setPostureReference(const cc::JointState &ref)
  {
    posture_task_->setReference(ics::to_sample(ref));
  }

  //////////////////////////////////////////////////////////////////////////
  // get states
  //////////////////////////////////////////////////////////////////////////

  const cc::JointState& MobileManipulatorFormulation::jointState()
  {
    auto& robot = idyn_.robot().wrapper();
    q_from_tsid(idyn_.q(), joint_state_.pos());
    joint_state_.vel() = idyn_.v().tail(robot.na());
    joint_state_.acc() = idyn_.a().tail(robot.na());
    return joint_state_;
  }

  const cc::CartesianState& MobileManipulatorFormulation::baseState()
  {
    // base_state_ = ics::to_cartesian_state(
    //   idyn_.robot().frameState(idyn_.data(), base_task_->frame_id()));
    base_state_.setZero();
    return base_state_;
  }

  cc::CartesianPosition MobileManipulatorFormulation::framePosition(const std::string& name) const
  {
    return ics::to_pose(idyn_.robot().framePosition(idyn_.data(), name));
  }

  cc::CartesianPosition MobileManipulatorFormulation::framePosition(pinocchio::FrameIndex id) const
  {
    return ics::to_pose(idyn_.robot().framePosition(idyn_.data(), id));
  }

  cc::CartesianState MobileManipulatorFormulation::frameState(const std::string& name) const
  {
    return ics::to_cartesian_state(idyn_.robot().frameState(idyn_.data(), name));
  }

  cc::CartesianState MobileManipulatorFormulation::frameState(pinocchio::FrameIndex id) const
  {
    return ics::to_cartesian_state(idyn_.robot().frameState(idyn_.data(), id));
  }

  //////////////////////////////////////////////////////////////////////////////
  // load / unload task
  //////////////////////////////////////////////////////////////////////////////

  bool MobileManipulatorFormulation::loadTasksFromParameters(
    ros::NodeHandle& nh, const std::vector<std::string> &names, bool activate, bool verbose)
  {
    if(names.empty())
      return true;
    if(!ics::TaskFactory::LoadList(idyn_, nh, names, activate, verbose))
    {
      PRINT_ERROR("Can't load tasks");
      return false;
    }
    if(!idyn_.resizeSolver(verbose))
    {
      PRINT_ERROR("Error resizing solver");
      return false;
    }
    return true;
  }

  bool MobileManipulatorFormulation::unLoadTasks(const std::vector<std::string> &names, bool verbose)
  {
    if(names.empty())
      return true;
    for(auto name : names)
    {
      if(idyn_.hasTask(name))
      {
        idyn_.unloadTask(name);
      }
      else if(idyn_.hasContact(name))
      {
        idyn_.unloadContact(name);
      }
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  // debug
  //////////////////////////////////////////////////////////////////////////////

  std::string MobileManipulatorFormulation::toString() const 
  {
    std::stringstream ss;
    ss << "MobileManipulatorFormulation References:" << std::endl;
    ss << "- q_tsid:\t" << idyn_.q().transpose() << std::endl;
    //ss << "- base:\t" << ics::to_cartesian_state(base_task_->getReference()).pos().toString() << std::endl;
    ss << "- posture:\t" << posture_task_->getReference().getValue().transpose() << std::endl;
    for(auto id : cc::BodyId::HandIds()) 
    {
      ss << "- " << cc::BodyId::Names()[id] << "_motion:\t" << bodies_[id]->motionReference().pos().toString() << std::endl;
      ss << "- " << cc::BodyId::Names()[id] << "_contact:\t" << bodies_[id]->motionPosition().toString() << std::endl;
    }
    return ss.str();
  }

  void MobileManipulatorFormulation::q_to_tsid(const cc::JointPosition& q, cc::VectorX& q_tsid)
  {
    // convert from q [x,y,th,q] to tsid [x,y,s,c,q] format
    if(idyn_.robot().hasMobileBase())
    {
      q_tsid.head(2) = q.head(2);
      q_tsid.segment(2,2) = cc::polar_to_cart(q[2]);
      q_tsid.tail(q.size()-3) = q.tail(q.size()-3);
    }
    else
      q_tsid = q;
  }

  void MobileManipulatorFormulation::q_from_tsid(const cc::VectorX& q_tsid, cc::JointPosition& q)
  {
    // convert from tsid [x,y,s,c,q] to q [x,y,th,q] format
    if(idyn_.robot().hasMobileBase())
    {
      q.head(2) = q_tsid.head(2);
      q[2] = cc::cart_to_polar(q_tsid[3], q_tsid[2]);
      q.tail(q.size()-3) = q_tsid.tail(q.size()-3);
    }
    else
      q = q_tsid;
  }

}