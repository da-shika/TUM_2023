#ifndef TOMM_CONTROL_PLUGIN_DIRECT_CONTROL_PLUGIN_H_
#define TOMM_CONTROL_PLUGIN_DIRECT_CONTROL_PLUGIN_H_

////////////////////////////////////////////////////////////////////////////////
// tomm_control_plugin includes
////////////////////////////////////////////////////////////////////////////////
#include <tomm_control_plugin/control_plugin_base.h>

////////////////////////////////////////////////////////////////////////////////
// module includes
////////////////////////////////////////////////////////////////////////////////
#include <tomm_application/app.h>
#include <tomm_hardware_interface/comm_interface.h>

namespace tomm_control_plugin
{
  /**
   * @brief TOMMControlPluginSim Class
   * 
   * This class is loaded by the ros_control framework to interact with the 
   * simulator or real robot. It directly spawns the remote client inside the 
   * controller plugin (single threaded).
   * 
   */
  class TOMMControlPlugin : public TOMMControlPluginBase
  {
  public:
    typedef TOMMControlPluginBase Base;

  private: 
    cc::Scalar sim_dt_;                         // simlation step size
    int sim_step_factor_;                       // gazebo step size over simulation step size
    ros::Duration period_;
    ros::Time start_time_, prev_time_;
    
    cc::JointPosition start_joint_pos_; 

    ////////////////////////////////////////////////////////////////////////////
    // modules
    ////////////////////////////////////////////////////////////////////////////
    std::unique_ptr<tomm::CommInterface> comm_interface_;
    std::unique_ptr<tomm::App> app_;

  public:
    TOMMControlPlugin();
    virtual ~TOMMControlPlugin();

  protected:
    virtual bool internalInit(
      ros::NodeHandle &root_nh,
      ros::NodeHandle &controller_nh,
      ClaimedResources &claimed_resources) override;

    virtual void internalStarting(const ros::Time &time) override;

    virtual void internalUpdate(const ros::Time &time, const ros::Duration &period) override;

    virtual void internalStopping(const ros::Time &time) override;

  private:
    void controllerUpdate(const ros::Time &time, const ros::Duration &period);

  };

  PLUGINLIB_EXPORT_CLASS(tomm_control_plugin::TOMMControlPlugin, controller_interface::ControllerBase)
}

#endif