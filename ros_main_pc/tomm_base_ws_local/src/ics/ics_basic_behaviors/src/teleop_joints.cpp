#include <pinocchio/fwd.hpp>

#include <ics_basic_behaviors/teleop_joints.h>

namespace ics
{

  TeleopJoints::TeleopJoints(const std::string& name) : 
    Base{name},
    SM{S::IDEL, S::N_STATES, E::N_EVENTS}
  {
    SM::add({
      {S::IDEL, S::TRACKING, E::CLOSE_TO_TARGET},
      {S::TRACKING, S::IDEL, E::TIMEOUT},
    });
  }

  TeleopJoints::~TeleopJoints()
  {
  }

  bool TeleopJoints::init(ros::NodeHandle &nh, Base::Parameters &global_params)
  {
    ////////////////////////////////////////////////////////////////////////////
    // load parameters
    ////////////////////////////////////////////////////////////////////////////
    cc::Parameters param(nh, Base::name());
    param.addRequired<std::string>("topic");
    if(!param.load())
    {
      PRINT_ERROR("Error loading parameter");
      return false;
    }
    
    target_sub_ = nh.subscribe(param.get<std::string>("topic"), 1, &TeleopJoints::teleopCallback, this);

    SM::reset(S::IDEL);
    return true;
  }

  bool TeleopJoints::start(const ros::Time &time)
  {
    ref_ = controller_->formulation().postureState();
    return true;
  }

  bool TeleopJoints::update(const ros::Time &time, const ros::Duration &period)
  {
    if(SM::is(S::TRACKING))
    {
      // check if still valid
      if((time - callback_time_) > ros::Duration(0.6))
      {
        SM::handle(E::TIMEOUT);
      }

      // update controller
      controller_->formulation().setPostureReference(ref_);
    }
    return true;
  }

  void TeleopJoints::teleopCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    if(SM::is(S::IDEL))
    {
      // check that size match
      int na = controller_->formulation().idyn().robot().wrapper().na();
      if(msg->name.size() != na || msg->position.size() != na)
      {
        ROS_ERROR_THROTTLE(1.0, "TeleopJoints::teleopCallback: wrong size");
        return;
      }
      
      // check that joint order matches
      std::vector<std::string> names = controller_->formulation().idyn().robot().actuatedJointNames();
      for(size_t i = 0; i < na; ++i)
      {
        if(msg->name[i] != names[i])
        {
          ROS_ERROR_THROTTLE(1.0, "TeleopJoints::teleopCallback: wrong joint names");
          return;
        }
      }

      // check distance to current posture
      for(size_t i = 0; i < na; ++i)
      {
        if(std::abs(msg->position[i] == controller_->formulation().postureState().pos()[i]) > 0.1)
        {
          return;
        }
      }

      // we accept
      SM::handle(E::CLOSE_TO_TARGET);
    }

    if(SM::is(S::TRACKING))
    {
      ref_.pos() = msg->position;
      ref_.vel().setZero();
      ref_.acc().setZero();
    }
  }

}