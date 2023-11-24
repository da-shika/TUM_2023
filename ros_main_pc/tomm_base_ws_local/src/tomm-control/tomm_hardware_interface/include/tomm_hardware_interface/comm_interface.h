#ifndef TOMM_HARDWARE_INTERFACE_COMM_INTERFACE_H_
#define TOMM_HARDWARE_INTERFACE_COMM_INTERFACE_H_

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/interfaces/module_base.h>
#include <tomm_interfaces/communication_interface.h>

namespace tomm
{

  /**
   * @brief CommInterface class
   * 
   */
  class CommInterface : public cc::ModuleBase, public CommunicationInterface
  {
    public:
      typedef cc::ModuleBase Base;

      enum State {CONSTRUCTED, DISCONNECTED, CONNECTED};

    private:
      State state_;
      
      //////////////////////////////////////////////////////////////////////////
      // data
      //////////////////////////////////////////////////////////////////////////
      cc::JointState command_;
      cc::RobotState robot_state_;

    public: 
      CommInterface(const std::string& name) : 
        Base{name},
        state_{CONSTRUCTED}
      {
      }
      virtual ~CommInterface()
      {
      }

      bool state() const { return state_; }

      //////////////////////////////////////////////////////////////////////////
      // inferface
      //////////////////////////////////////////////////////////////////////////

      virtual bool isConnected() const override { return state_ == CONNECTED; }

      virtual const cc::RobotState& robotState() const override { return robot_state_; }
      virtual cc::RobotState& robotState() override { return robot_state_; }

      virtual const cc::JointState& command() const override { return command_; }
      virtual cc::JointState& command() override { return command_; };
    
    protected:
      bool init(ros::NodeHandle& nh, Base::Parameters& global_params) override
      {
        state_ = CONNECTED;
        return true;
      }

      void start(const ros::Time& time) override
      {
        state_ = CONNECTED;
      }

      bool update(const ros::Time& time, const ros::Duration& period) override
      {
        return true;
      }

      void stop(const ros::Time& time) override
      {
        state_ = DISCONNECTED;
      }
  };

}

#endif
