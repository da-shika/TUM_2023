#ifndef ICS_BASIC_BEHAVIORS_TELEOP_FRAME_H_
#define ICS_BASIC_BEHAVIORS_TELEOP_FRAME_H_

#include <ics_behavior/behavior_base.h>

#include <control_core/trajectory/state_trajectories.h>
#include <control_core/statemachine/tiny_statemachine.h>

#include <control_core_msgs/CartesianStateStamped.h>

namespace ics
{
  namespace teleop_frame 
  {
    enum StateId {INIT, SPLINE, IDEL, TRACKING, N_STATES};
    enum EventId {START_SPLINE, STOP_SPLINE, CLOSE_TO_TARGET, TIMEOUT, N_EVENTS};
  }

  /**
   * @brief TeleopFrame Class
   * 
   * Teleoperation of specified robot frame.
   * 
   */
  class TeleopFrame : 
    public ics::BehaviorBase, 
    public cc::StateMachine<TeleopFrame, teleop_frame::StateId, teleop_frame::EventId>
  {
    public:
      typedef ics::BehaviorBase Base;
      typedef teleop_frame::StateId S;
      typedef teleop_frame::EventId E;
      typedef cc::StateMachine<TeleopFrame, S, E> SM;
      typedef tsid::tasks::TaskSE3Equality Motion;

    private:
      bool use_inital_offset_;
      bool do_inital_spline_;
      bool do_state_tracking_;

      //////////////////////////////////////////////////////////////////////////
      // task
      //////////////////////////////////////////////////////////////////////////
      std::string task_name_;
      std::shared_ptr<Motion> task_;

      //////////////////////////////////////////////////////////////////////////
      // spline
      //////////////////////////////////////////////////////////////////////////
      cc::Scalar spline_period_;
      cc::CartesianPosition spline_goal_;
      std::unique_ptr<cc::CartesianStateSlerpTrajectory> spline_;

      //////////////////////////////////////////////////////////////////////////
      // ros connections
      //////////////////////////////////////////////////////////////////////////
      ros::Publisher pose_pub_;
      ros::Subscriber target_sub_;
      ros::Time callback_time_;

      cc::CartesianState ref_state_w_;
      cc::CartesianPosition X_offset_local_;

      //////////////////////////////////////////////////////////////////////////
      // messages
      //////////////////////////////////////////////////////////////////////////
      geometry_msgs::PoseStamped pose_msg_;

    public:
      TeleopFrame(const std::string& name);
      virtual ~TeleopFrame();

    protected:
      virtual bool init(ros::NodeHandle &nh, Base::Parameters &global_params) override;

      virtual bool start(const ros::Time &time) override;

      virtual bool update(const ros::Time &time, const ros::Duration &period) override;

    private:
      void teleopPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

      void teleopStateCallback(const control_core_msgs::CartesianStateStampedConstPtr& msg);

      void parseTopic(const std::string& topic_name);

  };

}

#endif