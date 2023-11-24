#ifndef TOMM_HARDWARE_ROBOT_ARM_COMM_INTERFACE_H_
#define TOMM_HARDWARE_ROBOT_ARM_COMM_INTERFACE_H_

// communication
#include <RtThreads/Thread.h>
#include <tomm_hardware_real/arm/arm_comm_interface.h>
#include <tomm_hardware_real/arm/arm_configs.h>

// fitlers
#include <tumtools/Math/ButterFilter2.h>

namespace tomm_hw
{
  bool param_load_vector(const std::string& name, size_t size, Eigen::VectorXd& v);

  class ArmInterface : public RtThreads::Thread
  {
  public:
    typedef Tum::Tools::Butter2 ButterWorth;

  public:
    ArmInterface(const std::string& config_file_path = "", const std::string& int_pid_params_path = "");
    virtual ~ArmInterface();

    /**
     * @brief true if comm thread running
     *
     * @return true
     * @return false
     */
    bool isRunning() const;

    /**
     * @brief true if valid jointstate
     *
     * @return true
     * @return false
     */
    bool hasValidState() const;

    /**
     * @brief inialize communication
     *
     * @param root_nh
     * @param hw_nh
     * @return true
     * @return false
     */
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& hw_nh);

    /**
     * @brief read jointstate, only valid if returns true
     *
     * @param jointstate
     * @return true
     * @return false
     */
    bool read(JointState<DOF_ARM>& jointstate, const ros::Time& time, const ros::Duration& dt);

    /**
     * @brief wrtie the jointstate, only valid if return true
     *
     * @param jointstate
     */
    bool write(const JointState<DOF_ARM>& jointstate, const ros::Time& time, const ros::Duration& dt);

    /**
     * @brief stops the communication thread
     *
     */
    void stop();

    /**
     * @brief access to configuration object
     *
     * @return const ArmConfig&
     */
    const ArmConfig& config() const;

  private:
    void run();

    bool updateReal(const ros::Time& time, const ros::Duration& dt);

    bool updateSim(const ros::Time& time, const ros::Duration& dt);

    bool checkValidJointstateCmd(const JointState<DOF_ARM>& js, const JointState<DOF_ARM>& js_prev);

    bool checkValidInitalJointPos(const VectorDOFArm& q);

  private:
    // flags
    bool is_thread_stopped_;
    bool do_thread_stop_;
    bool wait_for_valid_joint_pos_;
    bool is_first_iteration_;
    bool has_valid_state_;

    // configuation
    ArmConfig config_;

    // communication
    std::unique_ptr<ArmCommInterface> comm_interface_;

    // states
    JointState<DOF_ARM> js_init_;      // inital state when starting
    JointState<DOF_ARM> js_cmd_;       // commanded by write()
    JointState<DOF_ARM> js_real_;      // feedback of robot
    JointState<DOF_ARM> js_send_;      // send to the robot
    JointState<DOF_ARM> js_send_prev_; // send to the robot in prev cycle

    // time
    ros::Time time_start_;

    // internal pid gain
    VectorDOFArm Kp_;
    VectorDOFArm Kd_;
    VectorDOFArm Ki_;
    VectorDOFArm v_thresh_;

    // filters
    ButterWorth filter_q_real_, filter_q_virt_;
    ButterWorth filter_v_real_, filter_v_virt_;
    ButterWorth filter_delta_q_, filter_delta_v_;
    ButterWorth filter_v_comp_;

    // config file
    std::string config_file_path_;
    std::string int_pid_params_path_;

    // protection jointstate
    std::mutex js_mutex_;
  };

}

#endif