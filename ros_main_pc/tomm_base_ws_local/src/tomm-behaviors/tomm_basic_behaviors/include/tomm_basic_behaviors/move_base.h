#ifndef TOMM_BASIC_BEHAVIORS_MOVE_BASE_H_
#define TOMM_BASIC_BEHAVIORS_MOVE_BASE_H_

#include <tomm_whole_body_controller/whole_body_controller.h>

#include <ics_behavior/behavior_base.h>

#include <tomm_basic_behaviors/MoveBaseParameters.h>

namespace tomm
{
  /**
   * @brief MoveBase like interface for the mobile base
   * 
   * Mobile base motion with twist input.
   * Note: this is handled as a cartesian task with zero pos gain.
   */
  class MoveBase : public ics::BehaviorBase
  {
    public:
      typedef ics::BehaviorBase Base;
      typedef tsid::tasks::TaskSE3Equality Motion;

      typedef tomm_basic_behaviors::MoveBaseParameters Params;

    private:
      std::shared_ptr<WholeBodyController> wbc_;
      Params params_;

      //////////////////////////////////////////////////////////////////////////
      // task
      //////////////////////////////////////////////////////////////////////////
      std::shared_ptr<Motion> task_;
      cc::Vector6 Kp_;

      //////////////////////////////////////////////////////////////////////////
      // ros connections
      //////////////////////////////////////////////////////////////////////////
      ros::Subscriber cmd_sub_;
      ros::Time last_cmd_time_;
      ros::Duration stop_dur_;

      cc::CartesianState ref_state_w_;

    public:
      MoveBase(const std::string& name);
      virtual ~MoveBase();

    protected:
      virtual bool init(ros::NodeHandle &nh, Base::Parameters &global_params) override;

      virtual bool start(const ros::Time &time) override;

      virtual bool update(const ros::Time &time, const ros::Duration &period) override;

    private:
      void cmdCallback(const geometry_msgs::Twist& msg);

  };
}

#endif