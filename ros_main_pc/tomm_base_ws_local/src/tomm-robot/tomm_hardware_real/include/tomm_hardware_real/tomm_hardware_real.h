#ifndef TOMM_HARDWARE_REAL_H
#define TOMM_HARDWARE_REAL_H

// ros control
#include <ros/ros.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// communication
#include <tomm_hardware_real/arm/arm_interface.h>
#include <tomm_hardware_real/omnibase/omnibase_interface.h>

// msgs
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

namespace tomm_hw
{

  class TOMMHardwareReal : public hardware_interface::RobotHW
  {
  public:
    typedef hardware_interface::RobotHW Base;

  public:
    TOMMHardwareReal(const std::string &l_config_file_path,
                     const std::string &r_config_file_path,
                     const std::string &internal_pid_file_path,
                     const std::string &base_config_file_path,
                     bool verbose);
    virtual ~TOMMHardwareReal();

    bool hasValidConnection() const;

    virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh) override;

    bool start(const ros::Time &time);

    virtual void read(const ros::Time &time, const ros::Duration &dt) override;

    virtual void write(const ros::Time &time, const ros::Duration &dt) override;

    void writeZero(const ros::Time &time, const ros::Duration &dt);

    bool shutDown() const;

    void requestH1ShutDown();

  private:
    // flags
    bool verbose_; // whether to print while read and write
    bool has_valid_connection_;
    bool shut_down_;
    std::string l_config_file_path_, r_config_file_path_, 
                internal_pid_file_path_, base_config_file_path_;

    // ros
    ros::ServiceServer shutdown_server_;
    ros::ServiceClient shutdown_client_;

    // Jointstate
    JointState<DOF_ARM> l_js_, r_js_;
    JointState<DOF_ARM> l_js_cmd_, r_js_cmd_;
    JointState<DOF_BASE> base_js_;
    JointState<DOF_BASE> base_js_cmd_;

    // hardware interfaces
    hardware_interface::JointStateInterface js_interface_;
    hardware_interface::PosVelJointInterface pvj_interface_;

    // low level communication
    std::unique_ptr<ArmInterface> l_arm_interface_, r_arm_interface_;
    std::unique_ptr<OmnibaseInterface> omnibase_interface_;

  private:
    bool shutDownService(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res);
  };

}

#endif