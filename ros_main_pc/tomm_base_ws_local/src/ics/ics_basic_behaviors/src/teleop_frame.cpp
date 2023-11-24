#include <pinocchio/fwd.hpp>

#include <ics_basic_behaviors/teleop_frame.h>

namespace ics
{

  TeleopFrame::TeleopFrame(const std::string& name) : 
    Base{name},
    SM{S::INIT, S::N_STATES, E::N_EVENTS},
    task_{nullptr},
    use_inital_offset_{true},
    do_inital_spline_{false}
  {
    SM::add({
      {S::INIT, S::SPLINE, E::START_SPLINE},
      {S::SPLINE, S::IDEL, E::STOP_SPLINE},
      {S::IDEL, S::TRACKING, E::CLOSE_TO_TARGET},
      {S::TRACKING, S::IDEL, E::TIMEOUT}
    });
  }

  TeleopFrame::~TeleopFrame()
  {
  }

  bool TeleopFrame::init(ros::NodeHandle &nh, Base::Parameters &global_params)
  {
    ////////////////////////////////////////////////////////////////////////////
    // load parameters
    ////////////////////////////////////////////////////////////////////////////
    cc::Parameters param(nh, Base::name());
    param.addRequired<std::string>("topic");
    param.addRequired<std::string>("task_name");
    param.addOptional<cc::CartesianPosition>("initial_spline_goal_pose", cc::CartesianPosition::Identity());
    param.addOptional<cc::CartesianPosition>("target_offset_pose", cc::CartesianPosition::Identity());
    param.addOptional<cc::Scalar>("initial_spline_period", 0.0);
    if(!param.load())
    {
      PRINT_ERROR("Error loading parameter");
      return false;
    }
    param.get("target_offset_pose", X_offset_local_);
    param.get("initial_spline_period", spline_period_);
    param.get("initial_spline_goal_pose", spline_goal_);
    param.get("task_name", task_name_);

    // check if we need to use the inital rotation offset or if its provided
    if(param.isLoaded("target_offset_pose"))
      use_inital_offset_ = false;
    if(param.isLoaded("initial_spline_goal_pose") && spline_period_ > 0)
      do_inital_spline_ = true;

    ////////////////////////////////////////////////////////////////////////////
    // ros connections
    ////////////////////////////////////////////////////////////////////////////
    std::string topic_name = param.get<std::string>("topic");

    // check subscribing to pose or state topic
    parseTopic(topic_name);
    if (do_state_tracking_)
      target_sub_ = nh.subscribe(topic_name, 1, &TeleopFrame::teleopStateCallback, this);
    else
      target_sub_ = nh.subscribe(topic_name, 1, &TeleopFrame::teleopPoseCallback, this);

    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(Base::name() + "/pose", 1);
    pose_msg_.header.frame_id = "world";

    return true;
  }

  bool TeleopFrame::start(const ros::Time &time)
  {
    ////////////////////////////////////////////////////////////////////////////
    // activate task and set default reference
    ////////////////////////////////////////////////////////////////////////////
    if(!controller_->formulation().idyn().addTask(task_name_))
    {
      PRINT_ERROR("Error activating task '%s'", task_name_);
      return false;
    }
    task_ = controller_->formulation().idyn().se3Task(task_name_);
    if(!task_)
    {
      PRINT_ERROR("Error getting task '%s'", task_name_);
      return false;
    }
    ref_state_w_.setZero();
    ref_state_w_.pos() = controller_->formulation().framePosition(task_->frame_id());
    auto ref = ics::to_sample(ref_state_w_);
    task_->setReference(ref);

    ////////////////////////////////////////////////////////////////////////////
    // setup spline
    ////////////////////////////////////////////////////////////////////////////
    if(do_inital_spline_)
    {
      auto base_w = controller_->formulation().basePosition();
      auto goal_w = base_w*spline_goal_;
      spline_ = std::make_unique<cc::CartesianStateSlerpTrajectory>(
        spline_period_, ref_state_w_.pos(), goal_w);
      ROS_WARN_STREAM(task_name_ << " start spline, target " << goal_w.toString());
    }
    SM::reset(S::INIT);
    SM::handle(E::START_SPLINE);
    return true;
  }

  bool TeleopFrame::update(const ros::Time &time, const ros::Duration &period)
  {
    cc::Scalar elapsed = (time - Base::startTime()).toSec();

    if(SM::is(S::SPLINE))
    {
      if(spline_)
        ref_state_w_ = spline_->evaluate(elapsed);
      if(elapsed > spline_period_)
        SM::handle(E::STOP_SPLINE);
    }

    if(SM::is(S::SPLINE) || SM::is(S::TRACKING))
    {
      // check if still valid
      if((time - callback_time_) > ros::Duration(0.6))
      {
        SM::handle(E::TIMEOUT);
      }

      // update controller
      auto ref = ics::to_sample(ref_state_w_);
      task_->setReference(ref);

      // publish
      pose_msg_.header.stamp = time;
      pose_msg_.pose = ref_state_w_.pos();
      cc::publish_if_subscribed(pose_pub_, pose_msg_);
    }
    return true;
  }

  void TeleopFrame::teleopPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    callback_time_ = ros::Time::now();

    // get the target in msg frame
    cc::CartesianPosition X_msg_local;
    X_msg_local = msg->pose;

    // get teleop target in world frame    
    cc::CartesianPosition X_msg_world;
    if(controller_->formulation().idyn().robot().hasFrame(msg->header.frame_id))
    {
      auto X_local_w = controller_->formulation().framePosition(msg->header.frame_id);
      X_msg_world = X_local_w*X_msg_local;
    }
    else
      X_msg_world = X_msg_local;

    if(SM::is(S::IDEL))
    {
      // store inital rotation between robot and msg frame
      if(use_inital_offset_)
      {
        X_offset_local_ = X_msg_world.inverse() * ref_state_w_.pos();
        X_offset_local_.linear().setZero();
      }
      ref_state_w_.pos() = X_msg_world * X_offset_local_;
      
      // check if tracking can start
      if((task_->position().head(3) - ref_state_w_.pos().linear()).norm() < 0.05)
      {
        SM::handle(E::CLOSE_TO_TARGET);
      }

      ROS_WARN_STREAM_THROTTLE(0.5, "IDEL=" << ref_state_w_.pos().toString());
    }

    if(SM::is(S::TRACKING))
    {
      ref_state_w_.pos() = X_msg_world * X_offset_local_;
      ref_state_w_.vel().setZero();
      ref_state_w_.acc().setZero();
    }
  }

  void TeleopFrame::teleopStateCallback(const control_core_msgs::CartesianStateStampedConstPtr &msg)
  {
    callback_time_ = ros::Time::now();

    // get the target in msg frame
    cc::CartesianState target_local;
    target_local = msg->state;

    // get teleop target in world frame
    cc::CartesianPosition X_local_world{cc::CartesianPosition::Identity()};
    if (controller_->formulation().idyn().robot().hasFrame(msg->header.frame_id))
    {
      X_local_world = controller_->formulation().framePosition(msg->header.frame_id);
    }

    cc::CartesianPosition X_msg_world = X_local_world * target_local.pos();
    cc::Matrix6 Ad_msg_world = cc::adjointMatrixMotion(cc::LinearPosition::Zero(), X_msg_world.angular().toRotationMatrix());

    if (SM::is(S::IDEL))
    {
      // store inital rotation between robot and msg frame
      if (use_inital_offset_)
      {
        X_offset_local_ = X_msg_world.inverse() * ref_state_w_.pos();
        X_offset_local_.linear().setZero();
      }
      ref_state_w_.pos() = X_msg_world * X_offset_local_;

      // check if tracking can start
      if ((task_->position().head(3) - ref_state_w_.pos().linear()).norm() < 0.05)
      {
        SM::handle(E::CLOSE_TO_TARGET);
      }
    }

    if (SM::is(S::TRACKING))
    {
      ref_state_w_.pos() = X_msg_world * X_offset_local_;
      // transform vel and acc from body frame to world frame
      ref_state_w_.vel() = Ad_msg_world * target_local.vel();
      ref_state_w_.acc() = Ad_msg_world * target_local.acc();
    }
  }

  void TeleopFrame::parseTopic(const std::string& topic_name)
  {
    PRINT_INFO("Subscribing to teleop topic: %s", topic_name.c_str());
    // Check the subscribed topic is for Pose or CartesianState
    std::vector<std::string> tokens = cc::split(topic_name, '_');
    if (!tokens.back().compare("state"))
      do_state_tracking_ = true;
    else
      do_state_tracking_ = false;
  }
}