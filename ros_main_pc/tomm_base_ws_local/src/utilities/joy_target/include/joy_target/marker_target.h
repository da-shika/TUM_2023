////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/systems/mass_spring_damper.h>
#include <control_core/interfaces/module_base.h>
#include <control_core/ros/marker.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <sensor_msgs/Joy.h>
#include <control_core_msgs/CartesianStateStamped.h>
#include <control_core_msgs/SetCartesianPositionGoal.h>

////////////////////////////////////////////////////////////////////////////////
// ros includes
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <joy_target/MarkerTargetParameters.h>

////////////////////////////////////////////////////////////////////////////////
// markers includes
////////////////////////////////////////////////////////////////////////////////
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>

namespace joy_target
{
  struct Target
  {
    cc::MassSpringDamperSystem system;
    cc::CartesianPosition pose;
    ros::Publisher state_pub;
    ros::Publisher pose_pub;
  };

  /**
   * @brief MarkerTarget Class
   * 
   */
  class MarkerTarget : public cc::ModuleBase
  {
  public:
    typedef cc::ModuleBase Base;

    private:
      interactive_markers::InteractiveMarkerServer server_;
      MarkerTargetParameters params_;
      std::vector<Target> targets_;
      tf::TransformListener tf_listner_;

    public:
      MarkerTarget(const std::string& name);
      virtual ~MarkerTarget();

    protected:
      virtual bool init(ros::NodeHandle& nh, cc::Parameters& global_params) override;

      virtual bool update(const ros::Time& time, const ros::Duration& dt) override;

    private:
      void parseJoy(const std::vector<float>& axes, const std::vector<int>& buttons);

    private:
      void interactiveMarkerFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, size_t idx);
  };

}