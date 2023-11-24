#include <pinocchio/fwd.hpp>
#include <tomm_basic_behaviors/move_to_cartesian_mobile_manipulator.h>

#include <control_core/math.h>

namespace tomm
{

  MoveToCartesianMobileManipulator::MoveToCartesianMobileManipulator(const std::string &name) : Base(name, true)
  {
  }

  MoveToCartesianMobileManipulator::~MoveToCartesianMobileManipulator()
  {
  }

  bool MoveToCartesianMobileManipulator::init(ros::NodeHandle &nh, cc::Parameters &global_params)
  {
    wbc_ = std::dynamic_pointer_cast<WholeBodyController>(controller_);
    if (!wbc_)
    {
      PRINT_ERROR("Error casting controller");
      return false;
    }
    return true;
  }

  bool MoveToCartesianMobileManipulator::start(const ros::Time &time)
  {
    if (goal_.period.data <= 0)
    {
      PRINT_ERROR("period=%f < 0", goal_.period.data);
      return false;
    }

    // initialize cartesian spline for each task
    for (size_t i = 0; i < goal_.motion_tasks.size(); ++i)
    {
      std::string name = goal_.motion_tasks[i].data;
      auto task = wbc_->cmdRobot().formulation().idyn().se3Task(name);
      // check if we have a se3 motion task on that frame
      if (!task)
      {
        PRINT_WARN("Task '%s' not active, load it first", name.c_str());
        if (!wbc_->cmdRobot().formulation().loadTasksFromParameters(nh_, {name}, true, true))
        {
          PRINT_ERROR("Task '%s' failed to load", name.c_str());
          return false;
        }
        task = wbc_->cmdRobot().formulation().idyn().se3Task(name);
      }
      else
      {
        PRINT_WARN("Task '%s' activated", name.c_str());
        wbc_->cmdRobot().formulation().idyn().addTask(name);
      }

      // set current ref pose as start
      auto cur = position(name);

      // extract goal wrt world
      auto X_msg_world = cc::CartesianPosition::Identity();
      if (goal_.targets.header.frame_id != "world")
      {
        if (!wbc_->cmdRobot().formulation().idyn().robot().hasFrame(goal_.targets.header.frame_id))
        {
          PRINT_ERROR("Unknown goal frame='%s'", goal_.targets.header.frame_id.c_str());
          return false;
        }
        X_msg_world = ics::to_pose(wbc_->cmdRobot().formulation().idyn().robot().framePosition(
            wbc_->cmdRobot().formulation().idyn().data(), goal_.targets.header.frame_id));
      }
      cc::CartesianPosition X_goal_msg;
      X_goal_msg = goal_.targets.poses[i];
      auto target = X_msg_world * X_goal_msg;

      // setup inital splines
      CartesianSplinePtr spline = std::make_unique<cc::CartesianStateSlerpTrajectory>(
          goal_.period.data, cur, target);

      PRINT_WARN_STREAM("cur_=" << cur.toString());
      PRINT_WARN_STREAM("target_=" << target.toString());

      spline_map_[name] = spline;
    }

    // change task weights
    wbc_->cmdRobot().formulation().idyn().updateTaskWeight("posture_task", 1.0);
    wbc_->cmdRobot().formulation().idyn().updateTaskWeight("base_motion", 100.0);

    PRINT_INFO("started");
    return true;
  }

  bool MoveToCartesianMobileManipulator::update(const ros::Time &time, const ros::Duration &period)
  {
    // ============== Update task references ===================================
    cc::Scalar elapsed = (time - Base::startTime()).toSec();
    feedback_.real.header.stamp = time;
    feedback_.cmd.header.stamp = time;

    for (auto it = spline_map_.begin(); it != spline_map_.end(); ++it)
    {
      const std::string &name = it->first;
      auto &spline = it->second;

      auto task = wbc_->cmdRobot().formulation().idyn().se3Task(name);
      cc::CartesianState ref = spline->evaluate(elapsed);
      cc::CartesianPosition cur = position(name);

      // feedback msg
      feedback_.real.poses.push_back(cur);
      feedback_.cmd.poses.push_back(ref.pos());

      // update the task reference
      auto reference = ics::to_sample(ref);
      task->setReference(reference);
    }
    // check the goal
    if (elapsed > goal_.period.data)
    {
      result_.real = feedback_.real;
      result_.cmd = feedback_.cmd;
      server_->setSucceeded(result_);
      return true;
    }

    server_->publishFeedback(feedback_);
    // =========================================================================
    // update base reference to current pose
    updateBaseRef();

    // ============ Update base and posture task weight ratio ==================
    cc::Scalar w_base, w_posture;

    // get hand positions
    auto X_lhand_w = wbc_->cmdRobot().formulation().framePosition("hand_link_left");
    auto X_rhand_w = wbc_->cmdRobot().formulation().framePosition("hand_link_right");

    // calculate relative 2d distance between hands and torso
    cc::CartesianPosition X_torso_w = ics::to_pose(wbc_->cmdRobot().formulation().idyn().robot().framePosition(
        wbc_->cmdRobot().formulation().idyn().data(), "torso_link"));
    auto X_lhand_b = X_torso_w.inverse() * X_lhand_w;
    auto X_rhand_b = X_torso_w.inverse() * X_rhand_w;

    ROS_WARN_STREAM_THROTTLE(0.5, "X_lhand_b = " << X_lhand_b.transpose());
    ROS_WARN_STREAM_THROTTLE(0.5, "X_rhand_b = " << X_rhand_b.transpose());

    // 2d euclidean distance from hands to base   d = sqrt(x*x + y*y)
    cc::Scalar d_lhand_b = sqrt(pow(X_lhand_b[0], 2) + pow(X_lhand_b[1], 2));
    ROS_WARN_STREAM_THROTTLE(0.5, "d_lhand_b (before scaling) = " << d_lhand_b);
    scaling(d_lhand_b, 0.4, 0.6, 0.9, 1.1);
    ROS_WARN_STREAM_THROTTLE(0.5, "d_lhand_b (after scaling) = " << d_lhand_b);
    cc::Scalar d_rhand_b = sqrt(pow(X_rhand_b[0], 2) + pow(X_rhand_b[1], 2));
    ROS_WARN_STREAM_THROTTLE(0.5, "d_rhand_b (before scaling) = " << d_rhand_b);
    scaling(d_rhand_b, 0.4, 0.6, 0.9, 1.1);
    ROS_WARN_STREAM_THROTTLE(0.5, "d_rhand_b (after scaling) = " << d_rhand_b);

    // choose larger one
    auto d = std::max(d_lhand_b, d_rhand_b);
    ROS_WARN_STREAM_THROTTLE(0.5, "d = " << d);

    // check if out of threshold
    // d = 0, weight_ratio = 2; d = 1, weigh_ratio = -2
    cc::Scalar weight_ratio = cc::polyActivation(d, 0, 1, 2, -2);


    // weight_ratio = lg( w_p / w_b)
    w_posture = 1;
    w_base = std::pow(10.0, weight_ratio) * w_posture;

    ROS_WARN_STREAM_THROTTLE(0.5, "w_posture = " << w_posture);
    ROS_WARN_STREAM_THROTTLE(0.5, "w_base = " << w_base << "\n");

    // update task weights
    wbc_->cmdRobot().formulation().idyn().updateTaskWeight("base_motion", w_base);
    wbc_->cmdRobot().formulation().idyn().updateTaskWeight("posture_task", w_posture);

    return true;
  }

  bool MoveToCartesianMobileManipulator::stop(const ros::Time &time)
  {
    // update references to current position
    updateBaseRef();
    updatePostureRef();

    // reset to default task weights
    wbc_->cmdRobot().formulation().idyn().updateTaskWeight("posture_task", 1.0);
    wbc_->cmdRobot().formulation().idyn().updateTaskWeight("base_motion", 100.0);

    // remove motion tasks
    for (auto it = spline_map_.begin(); it != spline_map_.end(); ++it)
    {
      wbc_->cmdRobot().formulation().idyn().removeTask(it->first);
      PRINT_WARN("Task '%s' removed", it->first.c_str());
    }

    PRINT_INFO(wbc_->cmdRobot().formulation().idyn().toString().c_str());
    PRINT_INFO("stopped");
    return true;
  }

  cc::CartesianPosition MoveToCartesianMobileManipulator::position(const std::string &task_name)
  {
    auto task = wbc_->cmdRobot().formulation().idyn().se3Task(task_name);
    return ics::to_pose(wbc_->cmdRobot().formulation().idyn().robot().framePosition(
        wbc_->cmdRobot().formulation().idyn().data(), task->frame_id()));
  }

  void MoveToCartesianMobileManipulator::updatePostureRef()
  {
    auto posture_cur = wbc_->cmdRobot().formulation().postureState();
    wbc_->cmdRobot().formulation().setPostureReference(posture_cur);
    
    ROS_WARN_STREAM("Posture updated to reference: \n"
                    << posture_cur.pos().transpose());
  }

  void MoveToCartesianMobileManipulator::updateBaseRef()
  {
    cc::CartesianState ref{cc::CartesianState::Zero()};
    ref.pos() = wbc_->cmdRobot().formulation().basePosition();
    wbc_->cmdRobot().formulation().setBaseReference(ref);

    ROS_WARN_STREAM_THROTTLE(0.5, "Base updated to reference: \n"
                                      << ref.pos().transpose());
  }

  void MoveToCartesianMobileManipulator::scaling(cc::Scalar &dist,
                                                 cc::Scalar edge_l, cc::Scalar thresh_l,
                                                 cc::Scalar thresh_r, cc::Scalar edge_r)
  {
    // inside thresholds, dist = 0
    // between thresholds and edges, dist (0,1)
    // outside of edges, dist = 1

    if (dist >= thresh_l && dist <= thresh_r)
    {
      dist = 0.0;
    }
    else if (dist > thresh_r && dist < edge_r)
    {
      dist = fabs((dist - thresh_r) / (edge_r - thresh_r));
    }
    else if (dist < thresh_l && dist > edge_l)
    {
      dist = fabs((dist - thresh_l) / (edge_l - thresh_l));
    }
    else if (dist >= edge_r || dist <= edge_l)
    {
      dist = 1.0;
    }
  }
}
