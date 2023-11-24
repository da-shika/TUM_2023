#include <joy_target/joy_target.h>

#include <control_core/ros/ros.h>

namespace joy_target
{

  JoyTarget::JoyTarget(const std::string& name) : 
    Base(name)
  {
  }

  JoyTarget::~JoyTarget()
  {
  }

  bool JoyTarget::init(ros::NodeHandle& nh, cc::Parameters& global_params)
  {
    // load parameters
    if(!params_.fromParamServer(nh, Base::name()))
    {
      ROS_ERROR("JoyTarget::init(): Failed to load params");
      return false;
    }

    // setup ros
    joy_sub_ = nh.subscribe(params_.joy_topic, 1, &JoyTarget::joyCallback, this);
    target_state_pub_ = nh.advertise<control_core_msgs::CartesianStateStamped>(params_.topic + "_state", 1);
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(params_.topic + "_pose", 1);

    // initalization (eigher by frame or by pose parameter)
    cc::CartesianPosition inital_pose;
    if(!params_.target_frame.empty())
    {
      // lookup the transformation of target wrt parent
      if(!cc::listenTransformation(
        tf_listner_, params_.parent_frame, params_.target_frame, inital_pose))
      {
        ROS_ERROR("JoyTarget::init() failed to get intial transformation");
        return false;
      }
      target_frame_ = params_.target_frame + "_joy";
    }
    else if (!params_.target_pose.isApprox(cc::CartesianPosition::Identity()))
    {
      target_frame_ = "target_frame_joy";
      inital_pose = params_.target_pose;
    }
    else
    {
      ROS_ERROR("JoyTarget::init() Need eigher 'target_frame' or 'target_pose' for initalization");
      return false;
    }

    system_.init(params_);
    system_.reset(inital_pose);
    is_active_ = false;
    wrench_.setZero();
    return true;
  }

  bool JoyTarget::update(const ros::Time& time, const ros::Duration& dt)
  {
    system_.update(wrench_, dt);

    control_core_msgs::CartesianStateStamped state_msg;
    state_msg.header.frame_id = params_.parent_frame;
    state_msg.header.stamp = time;
    state_msg.state = system_.state();
    cc::publish_if_subscribed(target_state_pub_, state_msg);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = params_.parent_frame;
    pose_msg.header.stamp = time;
    pose_msg.pose = system_.state().pos();
    cc::publish_if_subscribed(target_pose_pub_, pose_msg);

    geometry_msgs::TransformStamped T_msg;
    T_msg.header.frame_id = params_.parent_frame;
    T_msg.child_frame_id = target_frame_;
    T_msg.header.stamp = time;
    T_msg.transform.rotation = system_.state().pos().angular();
    T_msg.transform.translation = system_.state().pos().linear();
    broadcaster_.sendTransform(T_msg);

    return true;
  }

  void JoyTarget::parseJoy(const std::vector<float>& axes, const std::vector<int>& buttons)
  {
    wrench_.setZero();
    if(buttons[DEAD_MAN_SWITCH] == 0 ) 
    {
      is_active_ = false;
      return;
    }

    wrench_.linear().x() = axes[LINEAR_X];
    wrench_.linear().y() = axes[LINEAR_Y];
    wrench_.linear().z() = axes[LINEAR_Z];                              
    wrench_.angular().x() = axes[ANGULAR_X];                                  // TODO: LATER
    wrench_.angular().y() = axes[ANGULAR_Y];
    wrench_.angular().z() = axes[ANGULAR_Z];
    is_active_ = true;
  }

  void JoyTarget::joyCallback(const sensor_msgs::JoyConstPtr& msg)
  {
    parseJoy(msg->axes, msg->buttons);
  }

}