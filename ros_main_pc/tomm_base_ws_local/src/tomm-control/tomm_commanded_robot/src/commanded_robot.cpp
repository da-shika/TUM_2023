#include <tomm_commanded_robot/commanded_robot.h>
#include <walking_core/math.h>

namespace tomm
{
  CommandedRobot::CommandedRobot(const std::string& name) : 
    Base(name)
  {
  }

  CommandedRobot::~CommandedRobot()
  {
  }

  bool CommandedRobot::init(ros::NodeHandle &nh, Base::Parameters &global_params)
  {
    //////////////////////////////////////////////////////////////////////////
    // robot description
    //////////////////////////////////////////////////////////////////////////
    robot_ = std::make_unique<ics::RobotWrapper>("robot");
    if(!robot_->initRequest(nh, global_params))
    {
      PRINT_ERROR("Error initalizing robot");
      return false;
    }

    //////////////////////////////////////////////////////////////////////////
    // stack of task
    //////////////////////////////////////////////////////////////////////////
    formulation_ = std::make_unique<MobileManipulatorFormulation>(*robot_, "formulation");
    if(!formulation_->initRequest(nh, global_params))
    {
      PRINT_ERROR("Error initalizing stack of task formulation");
      return false;
    }

    //////////////////////////////////////////////////////////////////////////
    // setup the default configurtion
    //////////////////////////////////////////////////////////////////////////
    auto home_posture = global_params.get<cc::JointPosition>("home_posture");
    if(home_posture.size() != robot_->wrapper().na())
    {
      PRINT_ERROR("Wrong number of dof in home_posture. Requred=%d but loaded=%ld", 
        robot_->wrapper().na(), home_posture.size());
      return false;
    }
    // set intial joint state (relative transformations are now valid)
    cc::JointState joinstate_init = cc::JointState::Zero(robot_->wrapper().na());
    joinstate_init.pos() = home_posture.wrap();
    formulation_->setState(joinstate_init, true);

    //////////////////////////////////////////////////////////////////////////
    // setup internal commaned states (update once)
    //////////////////////////////////////////////////////////////////////////
    state_.setZero(robot_->wrapper().na());
    // contact_zmp_.resize(4, cc::LinearPosition::Zero());
    for(auto id : cc::BodyId::HandIds())
    {
      state_.ftSensor(id).frame() = formulation_->body(id)->frameName();
      // contact_zmp_[id] = formulation_->body(id)->motionPosition().linear();             // TODO: Should be contact center!
    }
    
    updateStates();
    return true;
  }

  void CommandedRobot::start(
    const cc::JointState& joint_state,
    const ros::Time &time)
  {
    ////////////////////////////////////////////////////////////////////////////
    // set the commanded robot to match starting joint state
    ////////////////////////////////////////////////////////////////////////////
    formulation_->setState(joint_state, true);
    
    ////////////////////////////////////////////////////////////////////////////
    // update all tasks references to current state to avoid jumps
    ////////////////////////////////////////////////////////////////////////////
    formulation_->idyn().setAllReferencesToCurrent(true);
    updateStates();
  }

  void CommandedRobot::setState(
    const cc::JointState& joint_state, bool recompute)
  {
    formulation_->setState(joint_state, recompute);
  }

  bool CommandedRobot::update(
    const cc::JointState& joint_state,
    const ros::Time &time, const ros::Duration &period)
  {
    // update the idyn 
    bool ret = formulation_->update(joint_state, time, period);
    
    // update internal states
    updateStates();
    return ret;                                                           
  }

  void CommandedRobot::updateStates()
  {
    // static const cc::Vector3 normal_vec(0,0,1);

    // write solution in robot state
    state_.floatingBase() = formulation_->baseState();
    state_.joints() = formulation_->jointState();
    for(auto id : cc::BodyId::HandIds())
    {
      // robot state
      state_.ftSensor(id).wrench() = formulation_->body(id)->contactWrench();
      state_.body(id) = formulation_->body(id)->motionState();
    }

    // for(auto id : cc::BodyId::HandIds())
    // {
    //   // zmp (note, contact wrench is world oriented)
    //   contact_zmp_[id] = state_.body(id).pos().linear() + cc::wrench_cop(state_.ftSensor(id).wrench());
    // }
  }

  std::string CommandedRobot::toString() const
  {
    std::stringstream ss;
    ss << formulation_->toString();                                       
    ss << "CommandedRobot:" << std::endl;
    ss << "- base=\t" << baseState().pos().toString() << std::endl;
    ss << "- q=\t" << jointState().pos().toString() << std::endl;
    for(auto id : cc::BodyId::HandIds())
      ss << "- " << cc::BodyId::Names()[id] << "_pos=\t" << bodyState(id).pos().toString() << std::endl;
    return ss.str();
  }

}