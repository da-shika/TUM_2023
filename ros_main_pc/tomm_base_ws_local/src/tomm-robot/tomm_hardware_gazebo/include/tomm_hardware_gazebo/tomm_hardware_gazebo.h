#ifndef TOMM_HARDWARE_GAZEBO_H
#define TOMM_HARDWARE_GAZEBO_H

#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/ImuSensor.hh>

#include <gazebo_ros_control/robot_hw_sim.h>
//#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <tomm_hardware_gazebo/tomm_default_robot_hw_sim.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>

typedef Eigen::Isometry3d eMatrixHom;

namespace gazebo_ros_control
{

  class ForceTorqueSensorDefinition
  {
  public:
    gazebo::physics::JointPtr gazebo_joint;
    std::string sensorName;
    std::string sensorJointName;
    std::string sensorFrame;
    double force[3];
    double torque[3];
    eMatrixHom sensorTransform;

    ForceTorqueSensorDefinition(const std::string &name,
                                const std::string &sensor_joint_name,
                                const std::string &frame)
    {
      sensorName = name;
      sensorJointName = sensor_joint_name;
      sensorFrame = frame;
      for (size_t i = 0; i < 3; ++i)
      {
        force[i] = 0.;
        torque[i] = 0.;
      }
    }
  };
  typedef boost::shared_ptr<ForceTorqueSensorDefinition> ForceTorqueSensorDefinitionPtr;

  class TOMMHardwareGazebo : public TOMMDefaultRobotHWSim
  {
  public:
    typedef TOMMDefaultRobotHWSim Base;
  
  protected:
    size_t n_dof_joints_;

    // joint pos vel interface
    hardware_interface::PosVelJointInterface pvj_interface_;

    // force torque sensor
    hardware_interface::ForceTorqueSensorInterface ft_sensor_interface_;
    std::vector<ForceTorqueSensorDefinitionPtr> forceTorqueSensorDefinitions_;

    // mobilebase model
    std::vector<std::string> base_joint_names_;
    std::vector<double> base_joint_position_;
    std::vector<double> base_joint_velocity_;
    std::vector<double> base_joint_effort_;
    std::vector<double> base_joint_effort_command_;
    std::vector<double> base_joint_position_command_;
    std::vector<double> base_joint_velocity_command_;
    gazebo::physics::ModelPtr mobilebase_;

    // publish odometry
    ros::Publisher odometry_pub_;
    boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;

    std::string base_frame_;
    std::string odometry_frame_;
    std::string tf_prefix_;
    gazebo::common::Time last_odom_publish_time_;
    gazebo::common::Time last_cmd_msg_time_;
    double odom_dur_;

    nav_msgs::Odometry odom_;
    ignition::math::Pose3d last_odom_pose_;

  public:
    TOMMHardwareGazebo();
    virtual ~TOMMHardwareGazebo();

    // Simulation-specific
    bool initSim(const std::string &robot_ns,
                 ros::NodeHandle nh,
                 gazebo::physics::ModelPtr model,
                 const urdf::Model *const urdf_model,
                 std::vector<transmission_interface::TransmissionInfo> transmissions);
    void readSim(ros::Time time, ros::Duration period);

    void writeSim(ros::Time time, ros::Duration period);

  private:
    bool parseForceTorqueSensors(ros::NodeHandle &nh,
                                 gazebo::physics::ModelPtr model,
                                 const urdf::Model *const urdf_model);

    bool parseMobileBase(ros::NodeHandle &nh,
                         gazebo::physics::ModelPtr model,
                         const urdf::Model *const urdf_model);

    void publishOdometry(const ros::Time &time, double step_time);
  };

}

#endif // TOMM_HARDWARE_GAZEBO_H
