#include <tomm_hardware_interface/tomm_interface.h>

namespace tomm
{

  TOMMInterface::TOMMInterface(
    CommunicationInterface& comm_interface,
    const std::string& name) : 
      Base{name},
      comm_interface_{comm_interface}
  {
  }

  TOMMInterface::~TOMMInterface()
  {
  }

  bool TOMMInterface::init(ros::NodeHandle& nh, Base::Parameters& global_params)
  {
    //////////////////////////////////////////////////////////////////////////
    // load parameters
    //////////////////////////////////////////////////////////////////////////
    if(!params_.fromParamServer(nh, Base::name()))
    {
      PRINT_ERROR("Can't load controller parameter");
      return false;
    }

    //////////////////////////////////////////////////////////////////////////
    // check if we have a connection
    //////////////////////////////////////////////////////////////////////////
    if(!comm_interface_.isConnected())
    {
      PRINT_ERROR("Communication interface not connected");
      return false;
    }
    auto& robot_state = comm_interface_.robotState();

    //////////////////////////////////////////////////////////////////////////
    // setup sensors
    //////////////////////////////////////////////////////////////////////////
    
    // joints
    std::size_t na = robot_state.joints().pos().size();    
    joint_state_cmd_.setZero(na);
    joint_state_last_cmd_.setZero(na);

    //////////////////////////////////////////////////////////////////////////
    // setup fitlers
    //////////////////////////////////////////////////////////////////////////

    // joint state differentiator
    joint_cmd_diff_ = std::make_unique<cc::StateDifferentiator<cc::JointState> >
      (cc::ScalarFiniteDifference::FirstOrderAccurarcyThree(
        global_params.get<cc::Scalar>("loop_rate")), na);
    joint_real_diff_ = std::make_unique<cc::StateDifferentiator<cc::JointState> >
      (cc::ScalarFiniteDifference::FirstOrderAccurarcyThree(
        global_params.get<cc::Scalar>("loop_rate")), na);

    has_safety_stop_ = false;
    return true;
  }

  void TOMMInterface::start(const ros::Time &time)
  {
    auto& robot_state = comm_interface_.robotState();

    //////////////////////////////////////////////////////////////////////////
    // update sensor readings once
    //////////////////////////////////////////////////////////////////////////
    update(time, ros::Duration(0));
    joint_state_cmd_ = robot_state.joints();
  }

  bool TOMMInterface::update(const ros::Time& time, const ros::Duration& period)
  {
    //////////////////////////////////////////////////////////////////////////
    // read robot state
    //////////////////////////////////////////////////////////////////////////
    auto& robot_state = comm_interface_.robotState();

    //////////////////////////////////////////////////////////////////////////
    // joint state
    //////////////////////////////////////////////////////////////////////////
    auto& q = robot_state.joints().pos();
    auto& v = robot_state.joints().vel();
    auto& q_last_cmd = joint_state_last_cmd_.pos();
    //robot_state_.joints() = state_real_diff_->update(robot_state_.joints());  // Numerical derivative
    joint_state_last_cmd_ = joint_state_cmd_;

    // check if there are any NANs inside the joint position
    if(q.hasNaN())
    {
      PRINT_ERROR("q.hasNaN() check");
      has_safety_stop_ = true;
      for(int i = 0; i < q.size(); ++i)
        if(std::isnan(q[i]))
          PRINT_ERROR("Joint has NAN in q[%d]=%f", i, q[i]);
      return false;
    }

    return true;
  }

  bool TOMMInterface::sendCommand(const cc::JointState& cmd)
  {
    //////////////////////////////////////////////////////////////////////////
    // Check command
    //////////////////////////////////////////////////////////////////////////

    if(has_safety_stop_)
    {
      PRINT_ERROR("Has has_safety_stop due to error");
      return false;
    }

    // check if there are any NANs inside the command
    auto& q_cmd = cmd.pos();
    if(q_cmd.hasNaN())
    {
      has_safety_stop_ = true;
      for(int i = 0; i < q_cmd.size(); ++i)
        if(std::isnan(q_cmd[i]))
          PRINT_ERROR("Joint has NAN in q_cmd[%d]=%f", i, q_cmd[i]);
      return false;
    }

    // check there are any jumps wrt the previous command
    cc::JointPosition q_delta = q_cmd - joint_state_cmd_.pos();
    if((q_delta.array().abs() > params_.joint_discontinuity_theshold).any())
    {
      if(std::abs(q_delta[2]) < params_.joint_discontinuity_theshold)                     // TODO: Dirty hack!
      {
        has_safety_stop_ = true;
        PRINT_ERROR_STREAM("q_cmd=\n" << q_cmd.toString());
        PRINT_ERROR_STREAM("q_prev_cmd=\n" << joint_state_cmd_.pos().toString());
        for(int i = 0; i < q_delta.size(); ++i)
          if(std::abs(q_delta[i]) > params_.joint_discontinuity_theshold)
            PRINT_ERROR("TOMMInterface::sendCommand(): Joint jump detected in q_delta[%d]=|%f| > %f", 
              i, q_delta[i], params_.joint_discontinuity_theshold);    
        return false;
      }
    }

    // compute the finite derivatives of commanded signal                                          
    joint_state_cmd_.pos() = q_cmd;
    joint_state_cmd_ = joint_cmd_diff_->update(joint_state_cmd_);
    
    //////////////////////////////////////////////////////////////////////////
    // write to comm interface
    //////////////////////////////////////////////////////////////////////////
    comm_interface_.command() = cmd;
    return true;
  }

  std::string TOMMInterface::toString() const
  {
    const auto& robot_state = comm_interface_.robotState();
    const auto& command = comm_interface_.command();
    
    std::stringstream ss;
    ss << "robot_state=\n" << robot_state.toString() << std::endl;
    ss << "command=\n" << command.toString() << std::endl;
    return ss.str();
  }

}