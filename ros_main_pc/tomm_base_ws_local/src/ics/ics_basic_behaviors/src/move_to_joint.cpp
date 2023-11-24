#include <pinocchio/fwd.hpp>
#include <ics_basic_behaviors/move_to_joint.h>

namespace ics
{

  MoveToJoint::MoveToJoint(const std::string& name) : 
    Base(name, true)
  {
  }

  MoveToJoint::~MoveToJoint()
  {
  }

  bool MoveToJoint::init(ros::NodeHandle &nh, Base::Parameters &global_params)
  {
    return true;
  }

  bool MoveToJoint::start(const ros::Time &time)
  {
    // get the posture task
    if(goal_.period.data <= 0)
    {
      ROS_ERROR("MoveToJoint::start(): period=%f < 0", goal_.period.data);
      return false;
    }

    // set current ref pose as start
    cur_ = controller_->formulation().postureState().pos();
    ref_.setZero(cur_.size());
    ref_.pos() = cur_;

    // load goals
    cc::JointPosition target, intermediate;
    target = goal_.target;
    intermediate = goal_.intermediate;
    if(intermediate.size() == 0)
        intermediate = 0.5*(target + cur_);

    PRINT_INFO_STREAM("target=\n" << target.toString());
    PRINT_INFO_STREAM("intermediate=\n" << intermediate.toString());
    PRINT_INFO_STREAM("cur_=\n" << cur_.toString());

    // setup inital splines
    spline_ = std::make_unique<Trajectory>(
      goal_.period.data, cur_, target, intermediate);

    PRINT_INFO_STREAM("MoveToJoint::start(): started");
    return true;
  }

  bool MoveToJoint::update(const ros::Time &time, const ros::Duration &period)
  {
    cc::Scalar elapsed = (time - Base::startTime()).toSec();
    ref_ = spline_->evaluate(elapsed);

    // pub feedback
    cur_ = controller_->formulation().postureState().pos();

    // check the goal
    if(elapsed > spline_->endTime())
    {
      result_.pose = feedback_.pose;
      server_->setSucceeded(result_);
    }

    // update
    controller_->formulation().setPostureReference(ref_);
    return true;
  }

  bool MoveToJoint::stop(const ros::Time &time)
  {
    ROS_INFO_STREAM("MoveToJoint::stop(): stopped");
    return true;
  }
}