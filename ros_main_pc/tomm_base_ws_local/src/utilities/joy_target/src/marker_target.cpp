#include <joy_target/marker_target.h>

#include <control_core/ros/ros.h>

namespace joy_target
{

  MarkerTarget::MarkerTarget(const std::string& name) : 
    Base(name),
    server_(name)
  {
  }

  MarkerTarget::~MarkerTarget()
  {
  }

  bool MarkerTarget::init(ros::NodeHandle& nh, cc::Parameters& global_params)
  {
    // load parameters
    if(!params_.fromParamServer(nh, Base::name()))
    {
      ROS_ERROR("MarkerTarget::init(): Failed to load params");
      return false;
    }

    size_t k = 0;
    for(auto frame : params_.frames)
    {
      Target target;

      target.pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        params_.topic_pre_fix + "/" + frame + "_pose", 1);
      target.state_pub = nh.advertise<control_core_msgs::CartesianStateStamped>(
        params_.topic_pre_fix + "/" + frame + "_state", 1);
      
      std::string frame_name = params_.tf_pre_fix + "/" + frame;
      if(cc::listenTransformation(tf_listner_, params_.parent_frame, frame_name, target.pose))
      {
        auto marker = cc::create_cube_marker(frame_name, 0, 0.05, 0.3, 0.3, 0.3, params_.parent_frame);
        auto int_marker = cc::create_interactive_marker_pos_orient(frame_name, marker, target.pose);
        int_marker.header = marker.header;
        server_.insert(int_marker, boost::bind(&MarkerTarget::interactiveMarkerFeedback, this, _1, k));
        server_.setPose(frame_name, target.pose, marker.header);
        server_.applyChanges();

        target.system.init(params_);
        target.system.reset(target.pose);
        targets_.push_back(target);      
        k++;

        ROS_WARN("Adding marker for exsisting frame: '%s'", frame_name.c_str());
      }
    }
    return true;
  }

  bool MarkerTarget::update(const ros::Time& time, const ros::Duration& dt)
  {
    for(auto& target : targets_)
    {
      cc::Wrench wrench = cc::cartesianErrorWorld(target.pose, target.system.state().pos());
      target.system.update(wrench, dt);
  
      control_core_msgs::CartesianStateStamped state_msg;
      state_msg.header.frame_id = params_.parent_frame;
      state_msg.header.stamp = time;
      state_msg.state = target.system.state();
      cc::publish_if_subscribed(target.state_pub, state_msg);

      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = params_.parent_frame;
      pose_msg.header.stamp = time;
      pose_msg.pose = target.system.state().pos();
      cc::publish_if_subscribed(target.pose_pub, pose_msg);
    }
    return true;
  }

  void MarkerTarget::interactiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg, size_t idx)
  {
    targets_[idx].pose = msg->pose;
  }

}