#include <pinocchio/fwd.hpp>
#include <ics_basic_behaviors/move_to_cartesian.h>

namespace ics
{

  MoveToCartesian::MoveToCartesian(const std::string& name) : 
    Base(name, true)
  {
  }

  MoveToCartesian::~MoveToCartesian()
  {
  }

  bool MoveToCartesian::init(ros::NodeHandle &nh, cc::Parameters &global_params)
  {
    return true;
  }

  bool MoveToCartesian::start(const ros::Time &time)
  {
    // check if we have a se3 motion task on that frame
    task_ = controller_->formulation().idyn().se3Task(goal_.motion_task.data);
    if(!task_)
    {
      PRINT_WARN("Task '%s' not active, load it first", goal_.motion_task.data.c_str());
      if(!controller_->formulation().loadTasksFromParameters(nh_, {goal_.motion_task.data}, true, true))
      {
        PRINT_ERROR("Task '%s' failed to load", goal_.motion_task.data.c_str());
        return false;
      }
      task_ = controller_->formulation().idyn().se3Task(goal_.motion_task.data);
    }
    else
    {
      PRINT_WARN("Task '%s' activated", goal_.motion_task.data.c_str());
      controller_->formulation().idyn().addTask(goal_.motion_task.data);
    }

    if(goal_.period.data <= 0)
    {
      PRINT_ERROR("period=%f < 0", goal_.period.data);
      return false;
    }

    // set current ref pose as start
    ref_.setZero();
    cur_ = position();

    // extract goal wrt world
    auto X_msg_world = cc::CartesianPosition::Identity();
    if(goal_.target.header.frame_id != "world")
    {
      if(!controller_->formulation().idyn().robot().hasFrame(goal_.target.header.frame_id))
      {
        PRINT_ERROR("Unknown goal frame='%s'", goal_.target.header.frame_id.c_str());
        return false;
      }
      X_msg_world = ics::to_pose(controller_->formulation().idyn().robot().framePosition(
        controller_->formulation().idyn().data(), goal_.target.header.frame_id));
    }
    cc::CartesianPosition X_goal_msg;
    X_goal_msg = goal_.target.pose;
    target_ = X_msg_world * X_goal_msg;

    // setup inital splines
    spline_ = std::make_unique<cc::CartesianStateSlerpTrajectory>(
      goal_.period.data, cur_, target_);

    PRINT_WARN_STREAM("cur_=" << cur_.toString());
    PRINT_WARN_STREAM("target_=" << target_.toString());
    PRINT_INFO("started");
    return true;
  }

  bool MoveToCartesian::update(const ros::Time &time, const ros::Duration &period)
  {
    cc::Scalar elapsed = (time - Base::startTime()).toSec();
    ref_ = spline_->evaluate(elapsed);

    // pub feedback
    cur_ = position();
    feedback_.real.pose = cur_;
    feedback_.cmd.pose = ref_.pos();
    feedback_.real.header.stamp = time;
    feedback_.cmd.header.stamp = time;
    server_->publishFeedback(feedback_);

    // check the goal
    if(elapsed > spline_->endTime())
    {
      result_.real = feedback_.real;
      result_.cmd = feedback_.cmd;
      server_->setSucceeded(result_);
    }

    // update the task references
    auto reference = ics::to_sample(ref_);
    task_->setReference(reference);

    return true;
  }

  bool MoveToCartesian::stop(const ros::Time &time)
  {
    PRINT_INFO("stopped");
    return true;
  }

  cc::CartesianPosition MoveToCartesian::position()
  {
    return ics::to_pose(controller_->formulation().idyn().robot().framePosition(
      controller_->formulation().idyn().data(), task_->frame_id()));
  }
}