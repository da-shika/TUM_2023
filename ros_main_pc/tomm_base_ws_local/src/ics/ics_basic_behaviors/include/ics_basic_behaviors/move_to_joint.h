#ifndef ICS_BASIC_BEHAVIORS_MOVE_JOINT_H_
#define ICS_BASIC_BEHAVIORS_MOVE_JOINT_H_

#include <ics_behavior/action_behavior_base.h>

#include <control_core/trajectory/state_trajectories.h>

#include <behavior_msgs/MoveToJointAction.h>

namespace ics
{

  /**
   * @brief MoveToJoint Behavior
   * 
   * Movements in Jointspace.
   * Note this behavior modifies the posture of the posture task.
   */
  class MoveToJoint : public ics::ActionBehaviorBase<behavior_msgs::MoveToJointAction>
  {
  public:
    typedef ActionBehaviorBase<behavior_msgs::MoveToJointAction> Base;
    typedef cc::JointStateSpline6 Trajectory;

  private:
    cc::JointPosition cur_;
    cc::JointState ref_;
    std::unique_ptr<Trajectory> spline_;

  public:
    MoveToJoint(const std::string& name);

    virtual ~MoveToJoint();

  protected:
    virtual bool init(ros::NodeHandle &nh, cc::Parameters &global_params) override;

    virtual bool start(const ros::Time &time) override;

    virtual bool update(const ros::Time &time, const ros::Duration &period) override;

    virtual bool stop(const ros::Time &time) override;

  };

}

#endif