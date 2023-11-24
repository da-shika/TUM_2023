#ifndef TOMM_HARDWARE_INTERFACE_TOMM_INTERFACE_H_
#define TOMM_HARDWARE_INTERFACE_TOMM_INTERFACE_H_

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types.h>
#include <control_core/algorithms.h>
#include <control_core/interfaces/module_base.h>


////////////////////////////////////////////////////////////////////////////////
// msgs includes
////////////////////////////////////////////////////////////////////////////////
#include <std_srvs/Empty.h>

#include <tomm_interfaces/hardware_interface.h>
#include <tomm_interfaces/communication_interface.h>
#include <tomm_hardware_interface/HardwareInterfaceParameters.h>

namespace tomm
{

  /**
   * @brief TOMMInterface Class
   * 
   * Used by the ros control plugin to read robot state and write 
   * joint commands.
   * 
   */
  class TOMMInterface : public cc::ModuleBase, public HardwareInterface
  {
    public:
      typedef cc::ModuleBase Base;
      typedef tomm_hardware_interface::HardwareInterfaceParameters Params;

      typedef cc::StateDifferentiator<cc::JointState> JointDifferentiator;
      typedef std::unique_ptr<JointDifferentiator> JointDifferentiatorPtr;

    private:
      //////////////////////////////////////////////////////////////////////////
      // parameters
      //////////////////////////////////////////////////////////////////////////
      bool has_safety_stop_;
      Params params_;

      //////////////////////////////////////////////////////////////////////////
      // communication interface
      //////////////////////////////////////////////////////////////////////////
      CommunicationInterface& comm_interface_;    //!< Communication to robot

      //////////////////////////////////////////////////////////////////////////
      // robot commands
      //////////////////////////////////////////////////////////////////////////
      cc::JointState joint_state_cmd_;            //!<  Commanded robot state.          // TODO no longer needed!
      cc::JointState joint_state_last_cmd_;       //!<  Last commanded joint statte

      //////////////////////////////////////////////////////////////////////////
      // Filtering, NumDiff
      //////////////////////////////////////////////////////////////////////////
      JointDifferentiatorPtr joint_cmd_diff_;   //!< Commanded joint derivative.
      JointDifferentiatorPtr joint_real_diff_;  //!< Real joint derivative

    public:
      TOMMInterface(
        CommunicationInterface& comm_interface, 
        const std::string& name="hardware_interface");

      virtual ~TOMMInterface();

      //////////////////////////////////////////////////////////////////////////
      // program flow functions
      //////////////////////////////////////////////////////////////////////////

      virtual bool sendCommand(const cc::JointState& cmd) override;

      //////////////////////////////////////////////////////////////////////////
      // hardware states
      //////////////////////////////////////////////////////////////////////////

      virtual const cc::JointState& jointState() const override { 
        return comm_interface_.robotState().joints(); }

      virtual const std::vector<cc::FtSensor>& ftSensors() const override {
        return comm_interface_.robotState().ftSensors(); }

      virtual const cc::JointState& commandedJointState() const override {
        return joint_state_cmd_; }

      virtual const cc::JointState& lastcommandedJointState() const override {
        return joint_state_last_cmd_; }

      virtual std::string toString() const override;

    protected:
      virtual bool init(ros::NodeHandle& nh, Base::Parameters& global_params) override;

      virtual void start(const ros::Time &time) override;

      virtual bool update(const ros::Time& time, const ros::Duration& period) override;

    private:
      bool shutdownHandler(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res); 

  };

}

#endif