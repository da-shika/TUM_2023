#ifndef ICS_BASIC_BEHAVIORS_MOVE_CARTESIAN_MOBILE_MANIPULATOR_H_
#define ICS_BASIC_BEHAVIORS_MOVE_CARTESIAN_MOBILE_MANIPULATOR_H_

#include <tomm_whole_body_controller/whole_body_controller.h>

#include <ics_behavior/action_behavior_base.h>

#include <control_core/trajectory/state_trajectories.h>
#include <control_core_msgs/CartesianStateStamped.h>

#include <behavior_msgs/MoveToCartesianMobileManipulatorAction.h>

namespace tomm
{

  /**
   * @brief MoveToCartesianMobileManipulator
   *
   * Support goals for multiple frames (e.g. goal for each hand)
   * Activate base motion (i.e. update posture/base task weights) when goals too far away
   *
   */
  class MoveToCartesianMobileManipulator : public ics::ActionBehaviorBase<behavior_msgs::MoveToCartesianMobileManipulatorAction>
  {
  public:
    typedef ActionBehaviorBase<behavior_msgs::MoveToCartesianMobileManipulatorAction> Base;
    typedef std::shared_ptr<cc::CartesianStateSlerpTrajectory> CartesianSplinePtr;
    typedef std::unordered_map<std::string, CartesianSplinePtr> CartesianSplineMap;

  private:
    std::shared_ptr<WholeBodyController> wbc_;
    CartesianSplineMap spline_map_;

  public:
    MoveToCartesianMobileManipulator(const std::string &name);

    virtual ~MoveToCartesianMobileManipulator();

  protected:
    virtual bool init(ros::NodeHandle &nh, cc::Parameters &global_params) override;

    virtual bool start(const ros::Time &time) override;

    virtual bool update(const ros::Time &time, const ros::Duration &period) override;

    virtual bool stop(const ros::Time &time) override;

  private:
    cc::CartesianPosition position(const std::string &task_name);

    /**
     * @brief Update reference for posture task to current value
     *
     */
    void updatePostureRef();

    /**
     * @brief Update reference for mobile base task to current value
     *
     */
    void updateBaseRef();

    /**
     * @brief Scaling the distance to be [0, 1]
     *
     * | dist <= edge_l | edge_l < dist < thresh_l | thresh_l <= dist <=  thresh_r | thresh_r  < dist <  edge_r | dist >= edge_r |
     * |       1        |         1 ~ 0            |              0                |            0 ~ 1           |       1        |
     *      *
     * @param dist
     * @param edge_l
     * @param thresh_l
     * @param thresh_r
     * @param edge_r
     */
    void scaling(cc::Scalar &dist,
                 cc::Scalar edge_l, cc::Scalar thresh_l,
                 cc::Scalar thresh_r, cc::Scalar edge_r);
  };

}

#endif
