#include <ics_tsid_wrapper/tsid_wrapper.h>

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/solvers/solver-HQP-factory.hpp>
//#include <tsid/solvers/solver-proxqp.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-fast.hpp>
#include <tsid/tasks/task-com-equality.hpp>

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/utilities/conversions.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/math.h>
#include <control_core/test_utilities/timing.h>

namespace ics
{
  TSIDWrapper::TSIDWrapper(Robot& robot, const std::string& name) : 
    Base(name),
    robot_(robot),
    verbose_(false)
  {
  }

  TSIDWrapper::~TSIDWrapper()
  {
  }

  bool TSIDWrapper::init(ros::NodeHandle &nh, Base::Parameters& global_params)
  {
    ////////////////////////////////////////////////////////////////////////////////
    // global parameter
    ////////////////////////////////////////////////////////////////////////////////
    dt_ref_ = 1./global_params.get<cc::Scalar>("loop_rate");

    ////////////////////////////////////////////////////////////////////////////
    // load parameters
    ////////////////////////////////////////////////////////////////////////////

    cc::Parameters parameter(nh, Base::name());
    parameter.addRequired<bool>("use_close_loop");
    if(!parameter.load())
    {
      PRINT_ERROR("Can't load parameter");
      return false;
    }
    parameter.get("use_close_loop", use_close_loop_);

    ////////////////////////////////////////////////////////////////////////////
    // initalize state variables
    ////////////////////////////////////////////////////////////////////////////

    t_ = 0;
    na_ = robot_.wrapper().na();
    nv_ = robot_.wrapper().nv();
    nq_ = robot_.wrapper().nq();

    q_.setZero(nq_);
    if(robot_.hasMobileBase())
      q_.head(4) << 0, 0, 1, 0;
    v_.setZero(nv_);
    a_sol_.setZero(nv_);

    ////////////////////////////////////////////////////////////////////////////
    // create the formulation
    ////////////////////////////////////////////////////////////////////////////

    formulation_ = std::make_unique<Formulation>("tsid", robot_.wrapper(), verbose_);
    formulation_->computeProblemData(0.0, q_, v_);

    ////////////////////////////////////////////////////////////////////////////
    // create geometry data 
    ////////////////////////////////////////////////////////////////////////////

    if(robot_.hasCollisionModel())
      geometry_data_ = pinocchio::GeometryData(robot_.geometryModel());

    ////////////////////////////////////////////////////////////////////////////
    // create the solver
    ////////////////////////////////////////////////////////////////////////////

    solver_.reset(tsid::solvers::SolverHQPFactory::createNewSolver(
      tsid::solvers::SOLVER_HQP_EIQUADPROG_FAST, "solver-eiquadprog"));
    solver_->resize(formulation_->nVar(), formulation_->nEq(), formulation_->nIn());
    solver_->setUseWarmStart(true);
    has_been_solved_ = false;

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  // managing of tasks
  //////////////////////////////////////////////////////////////////////////////

  bool TSIDWrapper::loadTask(const std::string& name, TaskPtr task, cc::Scalar weight, unsigned int priority, bool activate)
  {
    if(!task)
      return false;
    if(!cc::has(tasks_, name)) 
    {
      tasks_[name] = task;
      weights_[name] = weight;
    }
    else
      PRINT_WARN("'%s' already loaded, skipping.", name.c_str());
    if(activate)
      return addTask(name, weight, priority);
    return true;
  }

  bool TSIDWrapper::addTask(const std::string& name, cc::Scalar weight, unsigned int priority)
  {
    auto task = ics::get_task<tsid::tasks::TaskMotion>(tasks_, name, true);
    if(!task)
    {
      PRINT_WARN("'%s' does not exsits.", name.c_str());
      return false;
    }
    if(weight < 0)
    {
      weight = weights_[name];
    }
    weights_[name] = weight;
    if(weight < 0)
    {
      weight = 0; 
      priority = 0;
    }
    PRINT_WARN("task='%s', t_solver=%f, weight=%f, priority=%u", name.c_str(), t_, weight, priority);

    if(isActiveTask(task))
      return true;

    if(!formulation_->addMotionTask(*task, weight, priority))
    {
      PRINT_WARN("task='%s' could not be added.", name.c_str());
      return false;
    }
    return true;
  }

  bool TSIDWrapper::removeTask(const std::string& name, cc::Scalar transition_time)
  {
    auto task = ics::get_task<tsid::tasks::TaskBase>(tasks_, name, true);
    if(!task)
    {
      PRINT_WARN("'%s' does not exsits.", name.c_str());
      return false;
    }
    PRINT_WARN("task='%s', t_solver=%f", name.c_str(), t_);
    if(!formulation_->removeTask(name, transition_time))
    {
      PRINT_WARN("task='%s' could not be removed.", name.c_str());
      return false;
    }
    return true;
  }

  bool TSIDWrapper::unloadTask(const std::string& name)
  {
    if(!removeTask(name))
      return false;
    tasks_.erase(name);
    weights_.erase(name);
    PRINT_WARN("unloaded task='%s'", name.c_str());
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  // managing of contacts
  //////////////////////////////////////////////////////////////////////////////

  bool TSIDWrapper::loadContact(const std::string &name, ContactPtr contact, cc::Scalar transition_time, bool activate)
  {
    if(!contact)
      return false;
    if(!cc::has(contacts_, name))
    {
      contacts_[name] = contact;
      weights_[name] = contact->getMotionWeight();
    } 
    else
      PRINT_WARN("'%s' already loaded, skipping.", name.c_str());
    contacts_[name] = contact;
    if(activate)
      return addContact(name, transition_time);
    return true;
  }

  bool TSIDWrapper::addContact(const std::string& name, cc::Scalar transition_time)
  {
    auto contact = ics::get_contact(contacts_, name, false);
    if(!contact)
    {
      PRINT_WARN("'%s' does not exsits.", name.c_str());
      return false;
    }
    if(!hasActiveContact(name))
    {
      PRINT_WARN("contact='%s', t_solver=%f", name.c_str(), t_);
      if(!formulation_->addRigidContact(*contact, 
        contact->getRegularizationWeight(), 
        contact->getMotionWeight(), 
        contact->getPriorityLevel()))
      {
        PRINT_ERROR("contact='%s' failed", name.c_str());
        return false;
      }
      active_contact_names_.push_back(name);
      active_contacts_.push_back(contact);
    }
    return true;
  }

  bool TSIDWrapper::removeContact(const std::string& name, cc::Scalar transition_time)
  {
    auto contact = ics::get_contact(contacts_, name, false);
    if(!contact)
    {
      ROS_ERROR("'%s' does not exsits.", name.c_str());
      return false;
    }
    if(hasActiveContact(name))
    {
      PRINT_WARN("contact='%s', t_solver=%f", name.c_str(), t_);
      if(!formulation_->removeRigidContact(name, transition_time))
      {
        PRINT_ERROR("contact='%s' failed", name.c_str());
        return false;
      }
      cc::remove(active_contact_names_, name);
      cc::remove(active_contacts_, contact);
    }
    return true;
  }

  bool TSIDWrapper::unloadContact(const std::string &name)
  {
    if(!removeContact(name))
      return false;
    contacts_.erase(name);
    PRINT_WARN("unloaded contact='%s'", name.c_str());
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  // general tasks
  //////////////////////////////////////////////////////////////////////////////

  cc::Scalar TSIDWrapper::weight(const std::string& name) const
  {
    if(!cc::has(weights_, name))
    {
      PRINT_ERROR("'%s' does not exsits.", name.c_str());
      return -1;
    }
    return weights_.at(name);
  }

  bool TSIDWrapper::transitTaskWeight(const std::string &name, cc::Scalar goal_weight, const ros::Duration& dur)
  { 
    if(cc::has(task_weight_spline_map_, name))
      return false;
    TaskWeightSpline task_weight_spline;
    task_weight_spline.elapsed = 0.0;
    task_weight_spline.duration = dur.toSec();
    task_weight_spline.start_weight = weight(name);
    task_weight_spline.goal_weight = goal_weight;
    task_weight_spline_map_[name] = task_weight_spline;
    return true;
  }

  bool TSIDWrapper::updateTaskWeight(const std::string& task_name, cc::Scalar weight)
  {
    auto task = ics::get_task<tsid::tasks::TaskMotion>(tasks_, task_name, true);
    if(!task)
    {
      PRINT_ERROR("'%s' does not exsits.", task_name.c_str());
      return false;
    }
    weights_[task_name] = weight;
    formulation_->updateTaskWeight(task_name, weight);
    return true;
  }

  void TSIDWrapper::udpateTaskMask(const std::string& name, const cc::VectorX& mask)
  {
    // for some reason we need to remove first and than add back in
    removeTask(name);
    se3Task(name)->setMask(mask);
    addTask(name);
  }

  bool TSIDWrapper::setAllReferencesToCurrent(bool verbose)
  {
    // set motion task references
    for(auto item : formulation_->m_taskMotions)
    {
      if(dynamic_cast<tsid::tasks::TaskSE3Equality*>(&item->task))
      {
        auto ptr = dynamic_cast<tsid::tasks::TaskSE3Equality*>(&item->task);
        auto ref = ics::to_sample(robot_.framePosition(data(), ptr->frame_id()));
        ptr->setReference(ref);
        if(verbose)
          PRINT_INFO("task '%s'", ptr->name().c_str());
      }
      if(dynamic_cast<tsid::tasks::TaskComEquality*>(&item->task))
      {
        auto ptr = dynamic_cast<tsid::tasks::TaskComEquality*>(&item->task);
        auto ref = ics::to_sample(robot_.wrapper().com(data()));
        ptr->setReference(ref);
        if(verbose)
          PRINT_INFO("task '%s'", ptr->name().c_str());
      }
    }

    // set contact references
    for(auto contact : formulation_->m_contacts)
    {
      auto ptr = dynamic_cast<tsid::contacts::Contact6dExt*>(&contact->contact);
      if(ptr)
      {
        auto pos = robot_.framePosition(data(), ptr->getMotionTask().frame_id());
        auto ref = ics::to_sample(pos);
        ptr->setReference(pos);
        ptr->setMotionReference(ref);
        if(verbose)
          PRINT_INFO("contact '%s'", ptr->name().c_str());
      }
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  // SE3 tasks
  //////////////////////////////////////////////////////////////////////////////

  void TSIDWrapper::setSe3TaskReference(const pinocchio::SE3& ref, const std::string& name)
  {
    auto task = se3Task(name);
    if(task)
      task->setReference(ref);
  }

  void TSIDWrapper::setSe3TaskReference(TrajectorySample& sample, const std::string& task_name)
  {
    auto task = se3Task(task_name);
    if(task)
      task->setReference(sample);
  }

  tsid::trajectories::TrajectorySample TSIDWrapper::se3TaskReference(const std::string& name)
  {
    auto task = se3Task(name);
    return task->getReference();
  }

  pinocchio::SE3 TSIDWrapper::se3TaskPosition(const std::string& name)
  {
    auto task = se3Task(name);
    return robot_.framePosition(formulation_->data(), task->frame_id());
  }

  tsid::trajectories::TrajectorySample TSIDWrapper::se3TaskState(const std::string& name)
  {
    auto task = se3Task(name);
    return robot_.frameState(formulation_->data(), task->frame_id());
  }

  TSIDWrapper::TaskSE3EqualityPtr TSIDWrapper::se3TaskFromFrame(pinocchio::FrameIndex id)
  {
    for(auto it : tasks_)
    {
      auto task = std::dynamic_pointer_cast<tsid::tasks::TaskSE3Equality>(it.second);
      if(task)
        if(task->frame_id() == id)
          return task;
    }
    return nullptr;
  }

  TSIDWrapper::TaskSE3EqualityPtr TSIDWrapper::se3TaskFromFrame(const std::string& frame)
  {
    return se3TaskFromFrame(robot_.frameId(frame));
  }

  //////////////////////////////////////////////////////////////////////////////
  // Contact tasks
  //////////////////////////////////////////////////////////////////////////////

  cc::Wrench TSIDWrapper::contactWrenchSolution(const std::string& name)
  {
    if(!hasActiveContact(name))
    {
      // not active
      return cc::Wrench::Zero();
    }
    if(!has_been_solved_)
      return cc::Wrench::Zero();
    auto force = formulation_->getContactForces(name, solution_);
    auto contact = contactTask(name);
    return contact->getForceGeneratorMatrix()*force;
  }

  cc::Scalar TSIDWrapper::contactNormalForceSolution(const std::string& name)
  {
    if(!hasActiveContact(name))
    {
      // not active
      return 0.;
    }
    if(!has_been_solved_)
      return 0;
    auto force = formulation_->getContactForces(name, solution_);
    auto contact = contactTask(name);
    return contact->getNormalForce(force);
  }

  TSIDWrapper::ContactPtr TSIDWrapper::contactFromFrame(pinocchio::FrameIndex id)
  {
    for(auto it : contacts_)
    {
      auto contact = it.second;
      if(contact->getMotionTask().frame_id() == id)
        return contact;
    }
    return nullptr;
  }

  TSIDWrapper::ContactPtr TSIDWrapper::contactFromFrame(const std::string& frame)
  {
    return contactFromFrame(robot_.frameId(frame));
  }

  bool TSIDWrapper::updateRigidContactWeights(const std::string& name, cc::Scalar force_regularization_weight, cc::Scalar motion_weight)
  {
    auto contact = ics::get_contact(contacts_, name, false);
    if(!contact)
    {
      PRINT_ERROR("'%s' does not exsits.", name.c_str());
      return false;
    }
    contact->setRegularizationWeight(force_regularization_weight);
    if(motion_weight >= 0.0)
      contact->setMotionWeight(motion_weight);
    return formulation_->updateRigidContactWeights(name, force_regularization_weight, motion_weight);
  }

  bool TSIDWrapper::updateContactGains(const std::string& name, const cc::Vector6& Kp, const cc::VectorX& Kd)
  {
    auto contact = ics::get_contact(contacts_, name);
    if(!contact)
    {
      PRINT_ERROR("'%s' does not exsits.", name.c_str());
      return false;
    }
    if(Kp.size() == Kd.size())
    {
      contact->Kp(Kp);
      contact->Kd(Kd);
    }
    else
    {
      contact->Kp(Kp);
      contact->Kd(2.0*Kp.cwiseSqrt());
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Update and solve
  //////////////////////////////////////////////////////////////////////////////

  void TSIDWrapper::setState(const cc::VectorX& q, const cc::VectorX& v, bool recompute)
  {
    q_ = q;
    v_ = v;
    if(recompute)
      formulation_->computeProblemData(t_, q_, v_);
  }

  bool TSIDWrapper::update(
    const cc::VectorX& q, const cc::VectorX& v, 
    const ros::Time &time, const ros::Duration &period)
  {
    // solve qp
    bool ret;
    if(use_close_loop_)
      ret = solve(q, v, period.toSec());      // use external state (q,v)
    else
      ret = solve(q_, v_, period.toSec());    // use interal state (q_,v_)

    // update the geometry placement
    auto start_time = TIMENOW();
    robot_.updateGeometry(q_, formulation_->data(), geometry_data_);
    geometry_dur_ = DURATION(start_time);

    // update the task weight spline
    updateTaskWeightSplines(period.toSec());

    return ret;
  }

  //////////////////////////////////////////////////////////////////////////////
  // private function
  //////////////////////////////////////////////////////////////////////////////

  bool TSIDWrapper::solve(const cc::VectorX& q, const cc::VectorX& v, cc::Scalar dt)
  {
    if(q.hasNaN() || v.hasNaN())
    {
      PRINT_ERROR("Input contains NANs!");
      has_been_solved_ = false;
      return false;
    }

    // update dynamics using the input state (q, v). Can be real or commanded
    const auto& data = formulation_->computeProblemData(t_, q, v);

    // solve qp
    auto start_time = TIMENOW();
    solution_ = solver_->solve(data);
    solver_dur_ = DURATION(start_time);
    
    // integrate and update state
    if(solution_.status == tsid::solvers::HQP_STATUS_OPTIMAL)
    {
      a_sol_ = formulation_->getAccelerations(solution_);

      if(a_sol_.hasNaN())
      {
        PRINT_ERROR("Solution contains NANs!");
        has_been_solved_ = false;
        return false;
      }

      // integrate and update velocity and position signal inside (q_tsid_, v_tsid_)
      // using the acceleration computed with real states
      robot_.integrate(q, v, a_sol_, dt, q_, v_);
      t_ += dt;

      // restore the problem data to virtual state (all following computations will use virtual robot state)
      formulation_->computeProblemData(t_, q_, v_);
    }
    else
    {
      std::string error = "Status:";
      error += tsid::toString(solution_.status);
      switch (solution_.status) {
      case -1:
        error += " => Unknown";
        break;
      case 1:
        error += " => Infeasible ";
        break;
      case 2:
        error += " => Unbounded ";
        break;
      case 3:
        error += " => Max iter reached ";
        break;
      case 4:
        error += " => Error ";
        break;
      default:
        error += " => Uknown status";
      }
      error += " t_solver=" + std::to_string(t_);
      PRINT_ERROR("solve: %s", error.c_str());
      has_been_solved_ = false;
      return false;
    }
    has_been_solved_ = true;
    return true;
  }

  bool TSIDWrapper::resizeSolver(bool verbose)
  {
    // recreate
    solver_.reset(tsid::solvers::SolverHQPFactory::createNewSolver(
      tsid::solvers::SOLVER_HQP_EIQUADPROG_FAST, "solver-eiquadprog"));
    solver_->resize(formulation_->nVar(), formulation_->nEq(), formulation_->nIn());
    solver_->setUseWarmStart(true);

    // print new sizes
    if(verbose)
      ROS_INFO_STREAM(toString());
    return true;
  }

  void TSIDWrapper::updateTaskWeightSplines(cc::Scalar dt)
  {
    for (auto it = task_weight_spline_map_.begin(); it != task_weight_spline_map_.cend(); )
    {
      const auto& name = it->first;
      auto& spline = it->second;
      cc::Scalar s = spline.elapsed/spline.duration;
      cc::Scalar weight = cc::polyActivation(s, 0, 1, spline.start_weight, spline.goal_weight);
      updateTaskWeight(name, weight);
      spline.elapsed += dt;
      if(spline.elapsed > spline.duration)
        task_weight_spline_map_.erase(it++);
      else
        ++it;
    }
  }

  bool TSIDWrapper::isActiveTask(const TaskPtr& task) const
  {
    auto begin = formulation_->m_taskMotions.cbegin();
    auto end = formulation_->m_taskMotions.cend();
    auto it = std::find_if(begin, end, [&](const std::shared_ptr<tsid::TaskLevel>& task_base) {
        return &(task_base->task) == task.get(); });
    return it != formulation_->m_taskMotions.cend();
  }

  std::string TSIDWrapper::toString() const
  {
    std::stringstream ss;
    ss << "////////////////////////////////////////////////////////" << std::endl;
    ss << "/////////////////////// TSID ///////////////////////////" << std::endl;
    ss << "Num of variables : " << formulation_->nVar() << std::endl;
    ss << "Num of equalities : " << formulation_->nEq() << std::endl;
    ss << "Num of inequalities : " << formulation_->nIn() << std::endl;
    ss << "Loaded Tasks:" << std::endl;
    for(auto& task : tasks_)
      ss << "- " << task.first << std::endl;
    ss << "Active Tasks:" << std::endl;
    for(auto& task : formulation_->m_taskMotions)
      ss << "- " << task->task.name() << " w=" << weights_.at(task->task.name()) << std::endl;
    ss << "Loaded Contacts:" << std::endl;
    for(auto& contact : contacts_)
      ss << "- " << contact.first << std::endl;
    ss << "Active Contacts:" << std::endl;
    for(auto& contact : activeContacts())
      ss << "- " << contact->name() << " w=" << weights_.at(contact->name()) << std::endl;
    ss << "////////////////////////////////////////////////////////" << std::endl;
    return ss.str();
  }

}
