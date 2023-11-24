////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/systems/mass_spring_damper.h>
#include <control_core/interfaces/module_base.h>

////////////////////////////////////////////////////////////////////////////////
// msg includes
////////////////////////////////////////////////////////////////////////////////
#include <joy_target/JoyTargetParameters.h>
#include <sensor_msgs/Joy.h>
#include <control_core_msgs/CartesianStateStamped.h>
#include <control_core_msgs/SetCartesianPositionGoal.h>
#include <control_core_msgs/JoyKey.h>

////////////////////////////////////////////////////////////////////////////////
// ros includes
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace joy_target
{  
  /**
   * @brief Joy Target Class
   * 
   * Treat Joystick input as 6d acceleration input to 
   * mass spring damper system.
   * Publish resulting cartesian state and frame position.
   * Linear accelerations are given in fixed frame,
   * angular accelerations are given in body frame.
   * 
   */
  class JoyTarget : public cc::ModuleBase
  {
  public:
    typedef cc::ModuleBase Base;
    typedef control_core_msgs::JoyKey JOYKEY;

    enum JOYMAP {
        // motion
        LINEAR_X = JOYKEY::JOY_LY,  
        LINEAR_Y = JOYKEY::JOY_LX,
        LINEAR_Z = JOYKEY::JOY_RY,  // JOYKEY::ARROW_UP_DOWN, // 
        ANGULAR_X = JOYKEY::ARROW_UP_DOWN, // JOYKEY::ARROW_LEFT_RIGHT, //
        ANGULAR_Y = JOYKEY::JOY_RX, // JOYKEY::ARROW_LEFT_RIGHT, // JOYKEY::JOY_RY,
        ANGULAR_Z = JOYKEY::ARROW_LEFT_RIGHT,
        // deadman switch
        DEAD_MAN_SWITCH = JOYKEY::KEY_LB
    };

    private:
      bool is_active_;
      std::string target_frame_;
      tf::TransformListener tf_listner_;
      JoyTargetParameters params_;
      
      cc::MassSpringDamperSystem system_;
      cc::Wrench wrench_;
      
      ros::Subscriber joy_sub_;
      ros::Publisher target_state_pub_;
      ros::Publisher target_pose_pub_;
      tf::TransformBroadcaster broadcaster_;

    public:
      JoyTarget(const std::string& name);
      virtual ~JoyTarget();

    protected:
      virtual bool init(ros::NodeHandle& nh, cc::Parameters& global_params) override;

      virtual bool update(const ros::Time& time, const ros::Duration& dt) override;

    private:
      void parseJoy(const std::vector<float>& axes, const std::vector<int>& buttons);

    private:
      void joyCallback(const sensor_msgs::JoyConstPtr& msg);
  };

}