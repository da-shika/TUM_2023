#ifndef TOMM_WHOLE_BODY_CONTROLLER_WHOLE_BODY_CONTROLLER_H_
#define TOMM_WHOLE_BODY_CONTROLLER_WHOLE_BODY_CONTROLLER_H_

////////////////////////////////////////////////////////////////////////////////
// ics_formulation includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_formulation/controller_base.h>

////////////////////////////////////////////////////////////////////////////////
// hardware_interface includes
////////////////////////////////////////////////////////////////////////////////
#include <tomm_interfaces/hardware_interface.h>
#include <tomm_interfaces/whole_body_controller_interface.h>

////////////////////////////////////////////////////////////////////////////////
// modules
////////////////////////////////////////////////////////////////////////////////
#include <tomm_commanded_robot/commanded_robot.h>

namespace tomm
{

  /**
   * @brief WholeBodyController class
   * 
   * Unify real, commanded robot, state filters
   * 
   * Note: Here in tomm I'm not sure if real is requried
   */
  class WholeBodyController : 
    public ics::ControllerBase, 
    public tomm::WholeBodyControllerInterface
  {
    public:
      typedef ics::ControllerBase Base;
      // typedef h1_whole_body_controller::WholeBodyControllerParameters Params;
      // typedef h1_whole_body_controller::WholeBodyControllerServer Server;
      // typedef h1_whole_body_controller::WholeBodyControllerConfig Config;

    private:
      //////////////////////////////////////////////////////////////////////////
      // prameter
      //////////////////////////////////////////////////////////////////////////
      // Params params_;
      // std::unique_ptr<Server> reconfig_srv_;

      //////////////////////////////////////////////////////////////////////////
      // modules includes
      //////////////////////////////////////////////////////////////////////////
      HardwareInterface& hardware_interface_;
      
      std::unique_ptr<CommandedRobot> cmd_robot_;             // commanded robot (formulation)

      //////////////////////////////////////////////////////////////////////////
      // states
      //////////////////////////////////////////////////////////////////////////
      cc::JointState command_;

    public:
      WholeBodyController(HardwareInterface& hardware_interface, const std::string& name); 
      virtual ~WholeBodyController();

      //////////////////////////////////////////////////////////////////////////
      // access to modules
      //////////////////////////////////////////////////////////////////////////

      /**
       * @brief access to real robot
       */
      // const HumanoidInterface& realRobotInterface() const override { return *real_robot_; }
      // const RealRobot& realRobot() const { return *real_robot_; }
      // RealRobot& realRobot() { return *real_robot_; }

      /**
       * @brief access to commanded robot
       */
      const MobileManipulatorInterface& cmdRobotInterface() const override { return *cmd_robot_; }
      const CommandedRobot& cmdRobot() const { return *cmd_robot_; } 
      CommandedRobot& cmdRobot() { return *cmd_robot_; }

      //////////////////////////////////////////////////////////////////////////
      // ControllerBase interface
      //////////////////////////////////////////////////////////////////////////

      /**
       * @brief obtain the latest control command
       */
      virtual const cc::JointState& command() const override { return command_; }

      /**
       * @brief direct access to TSIDWrapper
       */
      virtual ics::FormulationInterface& formulation() { return cmd_robot_->formulation(); }
      virtual const ics::FormulationInterface& formulation() const { return cmd_robot_->formulation(); }

    protected:
      /**
       * @brief start the controller
       */
      virtual void start(const ros::Time &time) override;

      /**
       * @brief update the controller 
       */
      virtual bool update(const ros::Time &time, const ros::Duration &period) override;

      /**
       * @brief stop the controller
       */
      virtual void stop(const ros::Time &time) override;

      /**
       * @brief publish information
       */
      virtual void publish(const ros::Time &time) override;

      /** 
      * \brief initialize the Module after construction. 
      */
      virtual bool init(ros::NodeHandle& nh, cc::Parameters& global_params) override;

    // private:
    //   void reconfigureRequest(Config &config, uint32_t level);
  };

}

#endif