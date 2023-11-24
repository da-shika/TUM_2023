#ifndef ICS_BASIC_BEHAVIORS_MOVE_CARTESIAN_H_
#define ICS_BASIC_BEHAVIORS_MOVE_CARTESIAN_H_

#include <ics_behavior/action_behavior_base.h>

#include <control_core/trajectory/state_trajectories.h>
#include <control_core_msgs/CartesianStateStamped.h>

#include <behavior_msgs/MoveToCartesianAction.h>

namespace ics
{

  /**
   * @brief MoveToCartesian m
   * 
   */
  class MoveToCartesian : public ics::ActionBehaviorBase<behavior_msgs::MoveToCartesianAction>
  {
  public:
    typedef ActionBehaviorBase<behavior_msgs::MoveToCartesianAction> Base;

  private:
    std::shared_ptr<tsid::tasks::TaskSE3Equality> task_;

    cc::CartesianPosition target_;
    cc::CartesianPosition cur_;

    cc::CartesianState ref_;
    std::unique_ptr<cc::CartesianStateSlerpTrajectory> spline_;

  public:
    MoveToCartesian(const std::string& name);

    virtual ~MoveToCartesian();

    protected:
      virtual bool init(ros::NodeHandle &nh, cc::Parameters &global_params) override;

      virtual bool start(const ros::Time &time) override;

      virtual bool update(const ros::Time &time, const ros::Duration &period) override;

      virtual bool stop(const ros::Time &time) override;

    private:
      cc::CartesianPosition position();
  };

}

#endif