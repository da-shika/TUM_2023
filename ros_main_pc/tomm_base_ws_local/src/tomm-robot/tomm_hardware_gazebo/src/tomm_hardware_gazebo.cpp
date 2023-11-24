#include <cassert>
#include <boost/foreach.hpp>

#include <pluginlib/class_list_macros.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <transmission_interface/transmission_interface_loader.h>

#include <tomm_hardware_gazebo/tomm_hardware_gazebo.h>
#include <tomm_hardware_gazebo/utilities.h>

#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/gazebo_config.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

typedef Eigen::Vector3d eVector3;
typedef Eigen::Isometry3d eMatrixHom;
typedef Eigen::Matrix3d eMatrixRot;
typedef Eigen::Quaternion<double> eQuaternion;

using std::string;
using std::vector;

namespace gazebo_ros_control
{
  using namespace hardware_interface;

  TOMMHardwareGazebo::TOMMHardwareGazebo() : Base()
  {
  }

  TOMMHardwareGazebo::~TOMMHardwareGazebo()
  {
  }

  bool TOMMHardwareGazebo::initSim(const std::string &robot_ns, ros::NodeHandle nh,
                                   gazebo::physics::ModelPtr model,
                                   const urdf::Model *const urdf_model,
                                   std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    ROS_INFO_STREAM("Loading TOMM HARWARE GAZEBO");
    
    ////////////////////////////////////////////////////////////////////////////
    // register all default interfaces
    ////////////////////////////////////////////////////////////////////////////
    if (!Base::initSim(robot_ns, nh, model, urdf_model, transmissions))
    {
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // register a pos vel interface for all position handles
    ////////////////////////////////////////////////////////////////////////////
    for (unsigned int j = 0; j < n_dof_; j++)
    {
      if (transmissions[j].joints_.size() == 0)
        continue;
      else if (transmissions[j].joints_.size() > 1)
        continue;
      std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
      if (joint_interfaces.empty())
        continue;
      const std::string &hardware_interface = joint_interfaces.front();
      if (hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface")
      {
        pvj_interface_.registerHandle(hardware_interface::PosVelJointHandle(
            js_interface_.getHandle(joint_names_[j]), &joint_position_command_[j], &joint_velocity_command_[j]));
      }
    }
    registerInterface(&pvj_interface_);
    ROS_DEBUG_STREAM("Registered joint pos vel interface.");

    ////////////////////////////////////////////////////////////////////////////
    // register force torque sensors
    ////////////////////////////////////////////////////////////////////////////
    parseForceTorqueSensors(nh, model, urdf_model);
    for (size_t i = 0; i < forceTorqueSensorDefinitions_.size(); ++i)
    {
      ForceTorqueSensorDefinitionPtr &ft = forceTorqueSensorDefinitions_[i];
      ft_sensor_interface_.registerHandle(ForceTorqueSensorHandle(
          ft->sensorName, ft->sensorFrame, &ft->force[0], &ft->torque[0]));

      ROS_INFO("TOMMHardwareGazebo: Register: %s", ft->sensorName.c_str());
    }
    registerInterface(&ft_sensor_interface_);
    ROS_DEBUG_STREAM("Registered force-torque sensors.");

    ////////////////////////////////////////////////////////////////////////////
    // register additional joints for mobile base motion
    ////////////////////////////////////////////////////////////////////////////
    if (!parseMobileBase(nh, model, urdf_model))
    {
      ROS_ERROR("Failed parseMobileBase");
      return false;
    }
    base_joint_position_.resize(3);
    base_joint_velocity_.resize(3);
    base_joint_effort_.resize(3);
    base_joint_effort_command_.resize(3);
    base_joint_position_command_.resize(3);
    base_joint_velocity_command_.resize(3);
    for (size_t i = 0; i < 3; ++i)
    {
      // JointStateHandle
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          base_joint_names_[i], &base_joint_position_[i], &base_joint_velocity_[i], &base_joint_effort_[i]));
      // PosVelJointHandle
      pvj_interface_.registerHandle(hardware_interface::PosVelJointHandle(
          js_interface_.getHandle(base_joint_names_[i]), &base_joint_position_command_[i], &base_joint_velocity_command_[i]));
    }
    
    ////////////////////////////////////////////////////////////////////////////
    // set inital state
    ////////////////////////////////////////////////////////////////////////////
    double value = 0.0;
    for(size_t i = 0; i < n_dof_; ++i)
    {
      if(nh.getParam("zeros/" + joint_names_[i], value))
      {
        joint_position_[i] = value;
        joint_position_command_[i] = value;
        sim_joints_[i]->SetPosition(0, joint_position_command_[i]);
      }
    }
    std::string gzpose;
    if(nh.getParam("gzpose", gzpose))
    {
      eVector3 pose = extract_pose_gzpose(gzpose);
      ROS_WARN_STREAM("gzpose=" << pose.transpose());
      for(size_t i = 0; i < 3; ++i)
      {
        base_joint_position_[i] = pose[i];
        base_joint_position_command_[i] = pose[i];
      }
    }
    
    ////////////////////////////////////////////////////////////////////////////
    // setup variables
    ////////////////////////////////////////////////////////////////////////////
    last_odom_pose_ = mobilebase_->WorldPose();
    last_odom_publish_time_ = mobilebase_->GetWorld()->SimTime();
    last_cmd_msg_time_ = mobilebase_->GetWorld()->SimTime();
    tf_prefix_ = tf::getPrefixParam(nh);
    transform_broadcaster_.reset(new tf::TransformBroadcaster());

    return true;
  }

  void TOMMHardwareGazebo::readSim(ros::Time time, ros::Duration period)
  {
    gazebo::common::Time current_time = mobilebase_->GetWorld()->SimTime();

    // read all default resources
    Base::readSim(time, period);

    // read mobile base state position
    const ignition::math::Pose3d &pose = mobilebase_->WorldPose();
    base_joint_position_[0] = pose.Pos().X();
    base_joint_position_[1] = pose.Pos().Y();
    base_joint_position_[2] = pose.Rot().Yaw();

    const ignition::math::Vector3d &lin_vel = mobilebase_->WorldLinearVel();
    const ignition::math::Vector3d &ang_vel = mobilebase_->WorldAngularVel();
    base_joint_velocity_[0] = lin_vel[0];
    base_joint_velocity_[1] = lin_vel[1];
    base_joint_velocity_[2] = ang_vel[2];

    // Read force-torque sensors
    for (size_t i = 0; i < forceTorqueSensorDefinitions_.size(); ++i)
    {
      ForceTorqueSensorDefinitionPtr &ft = forceTorqueSensorDefinitions_[i];
      gazebo::physics::JointWrench ft_wrench = ft->gazebo_joint->GetForceTorque(0u);

      ft->force[0] = ft_wrench.body2Force.X();
      ft->force[1] = ft_wrench.body2Force.Y();
      ft->force[2] = ft_wrench.body2Force.Z();
      ft->torque[0] = ft_wrench.body2Torque.X();
      ft->torque[1] = ft_wrench.body2Torque.Y();
      ft->torque[2] = ft_wrench.body2Torque.Z();

      // Transform to sensor frame
      Eigen::MatrixXd transform(6, 6);
      transform.setZero();
      transform.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
      transform.block(3, 3, 3, 3) = ft->sensorTransform.rotation().transpose();
      eVector3 r = ft->sensorTransform.translation();
      transform.block(3, 0, 3, 3) = skew(r) * ft->sensorTransform.rotation().transpose();

      Eigen::VectorXd wrench(6);
      wrench << ft->torque[0], ft->torque[1], ft->torque[2], ft->force[0], ft->force[1],
          ft->force[2];
      Eigen::VectorXd transformedWrench(6);

      transformedWrench = transform * wrench;

      ft->torque[0] = transformedWrench(0);
      ft->torque[1] = transformedWrench(1);
      ft->torque[2] = transformedWrench(2);
      ft->force[0] = transformedWrench(3);
      ft->force[1] = transformedWrench(4);
      ft->force[2] = transformedWrench(5);
    }

    // publish the mobile base odom msg
    double elapsed = (current_time - last_odom_publish_time_).Double();
    if (elapsed > odom_dur_)
    {
      last_odom_publish_time_ = current_time;
      publishOdometry(time, elapsed);
    }
  }

  void TOMMHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
  {
    // write all default resources
    Base::writeSim(time, period);

    // real posiion
    double x = base_joint_position_[0];
    double y = base_joint_position_[1];
    double theta = base_joint_position_[2];

    // velocity
    double vx = base_joint_velocity_[0];
    double vy = base_joint_velocity_[1];
    double omega = base_joint_velocity_[2];

    // cmd position
    double x_cmd = base_joint_position_command_[0];
    double y_cmd = base_joint_position_command_[1];
    double theta_cmd = base_joint_position_command_[2];

    // set linear z and angular x, y to 0 to prevent base tilting
    double linear_vel_z = 0.0;  //mobilebase_->WorldLinearVel().Z();
    double angular_vel_x = 0.0;  //mobilebase_->WorldAngularVel().X();
    double angular_vel_y = 0.0;  //mobilebase_->WorldAngularVel().Y();
    double vx_cmd = base_joint_velocity_command_[0];
    double vy_cmd = base_joint_velocity_command_[1];
    double omega_cmd = base_joint_velocity_command_[2];

    double e_theta = theta_cmd - theta;
    e_theta = std::atan2(std::sin(e_theta), std::cos(e_theta));

    //write the floating base velcoity (world)
    mobilebase_->SetLinearVel(ignition::math::Vector3d(        
        vx_cmd * cosf(theta) - vy_cmd * sinf(theta) + 10.0*(x_cmd - x),
        vy_cmd * cosf(theta) + vx_cmd * sinf(theta) + 10.0*(y_cmd - y),
        linear_vel_z));
    mobilebase_->SetAngularVel(ignition::math::Vector3d(                        
        angular_vel_x,
        angular_vel_y,
        omega_cmd + 10.0*e_theta));
  }

  bool TOMMHardwareGazebo::parseForceTorqueSensors(ros::NodeHandle &nh,
                                                   gazebo::physics::ModelPtr model,
                                                   const urdf::Model *const urdf_model)
  {
    using std::string;
    using std::vector;

    const string ft_ns = "force_torque";
    vector<string> ft_ids = getIds(nh, ft_ns);
    ros::NodeHandle ft_nh(nh, ft_ns);
    typedef vector<string>::const_iterator Iterator;
    for (Iterator it = ft_ids.begin(); it != ft_ids.end(); ++it)
    {
      std::string sensor_name = *it;
      std::string sensor_joint_name;
      std::string sensor_frame_id;
      ros::NodeHandle ft_sensor_nh(ft_nh, sensor_name);
      xh::fetchParam(ft_sensor_nh, "frame", sensor_frame_id);
      xh::fetchParam(ft_sensor_nh, "sensor_joint", sensor_joint_name);

      ForceTorqueSensorDefinitionPtr ft(
          new ForceTorqueSensorDefinition(sensor_name, sensor_joint_name, sensor_frame_id));

      ft->gazebo_joint = model->GetJoint(ft->sensorJointName);

      // Get sensor parent transform
      std::shared_ptr<const urdf::Link> urdf_sensor_link;
      std::shared_ptr<const urdf::Joint> urdf_sensor_joint;
      urdf_sensor_link = urdf_model->getLink(ft->sensorFrame);
      urdf_sensor_joint = urdf_model->getJoint(sensor_joint_name);

      if (!urdf_sensor_link)
      {
        ROS_ERROR_STREAM("Problem finding link: " << ft->sensorFrame
                                                  << " to attach FT sensor in robot model");
        return false;
      }

      if (!urdf_sensor_joint)
      {
        ROS_ERROR_STREAM("Problem finding joint: " << ft->sensorJointName
                                                   << " to attach FT sensor in robot model");
        return false;
      }

      // Recursively follow the transform until the parent
      bool parentFound = false;
      eMatrixHom sensorTransform;
      sensorTransform.setIdentity();

      // Check that is not the actual first link
      if (urdf_sensor_link->name == urdf_sensor_joint->child_link_name)
      {
        parentFound = true;
      }

      while (!parentFound)
      {
        std::cerr << "      " << urdf_sensor_link->name << " - "
                  << urdf_sensor_joint->child_link_name << std::endl;

        urdf::Pose tf_urdf = urdf_sensor_link->parent_joint->parent_to_joint_origin_transform;
        eMatrixHom tf_eigen;
        convert(tf_urdf, tf_eigen);
        sensorTransform = tf_eigen * sensorTransform;

        urdf_sensor_link = urdf_sensor_link->getParent();

        if (urdf_sensor_joint->child_link_name == urdf_sensor_link->name)
        {
          parentFound = true;
        }
      }

      if (!parentFound)
      {
        ROS_ERROR_STREAM("No frame found for force torque sensor");
      }

      ft->sensorTransform = sensorTransform;

      if (!ft->gazebo_joint)
      {
        ROS_ERROR_STREAM("Could not find joint '"
                         << ft->sensorJointName << "' to which a force-torque sensor is attached.");
        return false;
      }

      forceTorqueSensorDefinitions_.push_back(ft);
      ROS_INFO_STREAM("Parsed fake FT sensor: " << sensor_name << " in frame: " << sensor_frame_id);
    }
    return true;
  }

  bool TOMMHardwareGazebo::parseMobileBase(ros::NodeHandle &nh,
                                           gazebo::physics::ModelPtr model,
                                           const urdf::Model *const urdf_model)
  {
    using std::string;
    using std::vector;

    const string mobile_base_ns = "mobile_base";
    ros::NodeHandle mb_nh(nh, mobile_base_ns); 

    // fetch the model and link
    xh::fetchParam(mb_nh, "base_frame", base_frame_);
    gazebo::physics::LinkPtr link = model->GetLink(base_frame_);
    if (!link)
    {
      ROS_ERROR("Cant find '%s' in gazebo structure", base_frame_.c_str());
      return false;
    }
    mobilebase_ = link->GetModel();

    // setup parameters
    base_joint_names_.resize(3);
    xh::fetchParam(mb_nh, "joint_x_name", base_joint_names_[0]);
    xh::fetchParam(mb_nh, "joint_y_name", base_joint_names_[1]);
    xh::fetchParam(mb_nh, "joint_theta_name", base_joint_names_[2]);

    xh::fetchParam(mb_nh, "odometry_frame", odometry_frame_);

    double rate = 200.0;
    xh::fetchParam(mb_nh, "odometry_rate", rate);
    odom_dur_ = 1. / rate;

    std::string odometry_topic;
    xh::fetchParam(mb_nh, "odometry_topic", odometry_topic);
    odometry_pub_ = nh.advertise<nav_msgs::Odometry>(odometry_topic, 1);

    return true;
  }

  void TOMMHardwareGazebo::publishOdometry(const ros::Time &time, double step_time)
  {
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_link_frame = tf::resolve(tf_prefix_, base_frame_);

    // getting data for base_link to odom transform
    ignition::math::Pose3d pose = mobilebase_->WorldPose();
    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
    tf::Transform base_link_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_link_to_odom, time, odom_frame, base_link_frame));

    // publish odom topic
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
    linear.X() = (pose.Pos().X() - last_odom_pose_.Pos().X()) / step_time;
    linear.Y() = (pose.Pos().Y() - last_odom_pose_.Pos().Y()) / step_time;
    double v_th = base_joint_velocity_command_[2];
    if (v_th > M_PI / step_time)
    {
      // we cannot calculate the angular velocity correctly
      odom_.twist.twist.angular.z = v_th;
    }
    else
    {
      float last_yaw = last_odom_pose_.Rot().Yaw();
      float current_yaw = pose.Rot().Yaw();
      while (current_yaw < last_yaw - M_PI)
        current_yaw += 2 * M_PI;
      while (current_yaw > last_yaw + M_PI)
        current_yaw -= 2 * M_PI;
      float angular_diff = current_yaw - last_yaw;
      odom_.twist.twist.angular.z = angular_diff / step_time;
    }
    last_odom_pose_ = pose;

    // convert velocity to child_frame_id (aka base_link)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    odom_.header.stamp = time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_link_frame;

    odometry_pub_.publish(odom_);
  }

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::TOMMHardwareGazebo, gazebo_ros_control::RobotHWSim)
