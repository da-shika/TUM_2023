#include <pinocchio/fwd.hpp>

#include <tomm_basic_behaviors/move_base.h>

#include <control_core/math.h>

namespace tomm
{

  MoveBase::MoveBase(const std::string& name) : 
    Base{name},
    task_{nullptr},
    Kp_{cc::Vector6::Zero()},
    stop_dur_{ros::Duration(0.2)}
  {
  }

  MoveBase::~MoveBase()
  {
  }

  bool MoveBase::init(ros::NodeHandle &nh, Base::Parameters &global_params)
  {
    wbc_ = std::dynamic_pointer_cast<WholeBodyController>(controller_);
    if(!wbc_)
    {
      PRINT_ERROR("Error casting controller");
      return false;
    }

    //////////////////////////////////////////////////////////////////////////
    // load parameters
    //////////////////////////////////////////////////////////////////////////
    if(!params_.fromParamServer(nh, Base::name()))
    {
      PRINT_ERROR("Can't load controller parameter");
      return false;
    }

    cmd_sub_ = nh.subscribe(params_.topic, 1, &MoveBase::cmdCallback, this);
    return true;
  }

  bool MoveBase::start(const ros::Time &time)
  {
    task_ = wbc_->cmdRobot().formulation().baseTask();

    // reset to current position
    ref_state_w_.setZero();
    ref_state_w_.pos() = wbc_->cmdRobot().formulation().basePosition();

    last_cmd_time_ = time;
    return true;
  }

  bool MoveBase::update(const ros::Time &time, const ros::Duration &period)
  {
    if(time - last_cmd_time_ > stop_dur_)
      ref_state_w_.vel().setZero();

    // integrate twist
    ref_state_w_.pos() = cc::integrateVelocityWorld(
      ref_state_w_.pos(), ref_state_w_.vel(), period.toSec());

    // send out
    wbc_->cmdRobot().formulation().setBaseReference(ref_state_w_);
    return true;
  }

  void MoveBase::cmdCallback(const geometry_msgs::Twist& msg)
  {
    ref_state_w_.vel() = msg;

    // express in world
    ref_state_w_.vel() = cc::changeRefFrame(
      ref_state_w_.vel(), ref_state_w_.pos());
    last_cmd_time_ = ros::Time::now();
  }

}