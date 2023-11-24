#ifndef ICS_BASIC_BEHAVIORS_TELEOP_JOINTS_H_
#define ICS_BASIC_BEHAVIORS_TELEOP_JOINTS_H_

#include <ics_behavior/behavior_base.h>

#include <control_core/trajectory/state_trajectories.h>
#include <control_core/statemachine/tiny_statemachine.h>

#include <control_core_msgs/CartesianStateStamped.h>

namespace ics
{
  namespace teleop_joints 
  {
    enum StateId {IDEL, TRACKING, N_STATES};
    enum EventId {CLOSE_TO_TARGET, TIMEOUT, N_EVENTS};
  }

  /**
   * @brief TeleopJoints Class
   * 
   * Teleoperation of specified robot frame.
   * 
   */
  class TeleopJoints : 
    public ics::BehaviorBase, 
    public cc::StateMachine<TeleopJoints, teleop_joints::StateId, teleop_joints::EventId>
  {
    public:
      typedef ics::BehaviorBase Base;
      typedef teleop_joints::StateId S;
      typedef teleop_joints::EventId E;
      typedef cc::StateMachine<TeleopJoints, S, E> SM;

    private:
      cc::JointState ref_;
      ros::Subscriber target_sub_;
      ros::Time callback_time_;

    public:
      TeleopJoints(const std::string& name);
      virtual ~TeleopJoints();

    protected:
      virtual bool init(ros::NodeHandle &nh, Base::Parameters &global_params) override;

      virtual bool start(const ros::Time &time) override;

      virtual bool update(const ros::Time &time, const ros::Duration &period) override;

    private:
      void teleopCallback(const sensor_msgs::JointStateConstPtr& msg);

  };

}

#endif