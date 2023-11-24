#ifndef TOMM_HARDWARE_BASE_INTERFACE_H_
#define TOMM_HARDWARE_BASE_INTERFACE_H_

// communication
#include <RtThreads/Thread.h>
#include <tomm_hardware_real/omnibase/omnibase_rt_driver.h>
#include <tomm_hardware_real/utilities/joint_state.h>
#include <tomm_hardware_real/omnibase/omnibase_configs.h>

// ros
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

namespace tomm_hw
{
  typedef RtThreads::Thread TThread;

  class OmnibaseInterface : public TThread
  {
  public:
    OmnibaseInterface(const std::string &config_file_path = "");
    virtual ~OmnibaseInterface();

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
    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh);

    /**
       * @brief read jointstate, only valid if returns true
       * 
       * @param jointstate 
       * @return true 
       * @return false 
       */
    bool read(JointState<DOF_BASE> &jointstate, const ros::Time &time, const ros::Duration &dt);

    /**
       * @brief wrtie the jointstate, only valid if return true
       * 
       * @param jointstate 
       */
    bool write(const JointState<DOF_BASE> &jointstate, const ros::Time &time, const ros::Duration &dt);

    /**
       * @brief stops the communication thread
       * 
       */
    void stop();

    /**
       * @brief access to configuration object
       * 
       * @return const BaseConfig& 
       */
    const OmnibaseConfig &config() const;

  private:
    void run();

    bool updateReal(const ros::Time &time, const ros::Duration &dt);

    bool updateSim(const ros::Time &time, const ros::Duration &dt);

    void checkValidCmd(VectorDOFBase &vel_cmd);

    bool checkValidInitalJointPos(const VectorDOFArm &q);

  private:
    /**
     * update current position, current velocity
     */
    void update_odometry(const omnibase_rt_driver::DataVec& data_vec, JointState<DOF_BASE>& js_curr, const ros::Duration& dt);

    /**
     * @brief simulate the position, velocity updates
     */
    void simulate_odometry(const Vector3& vel_cmd, JointState<DOF_BASE>& basestate, const ros::Duration& dt);

    /**
     * update wheel command
     */
    void update_driver(const Vector3& vel_cmd, Vector3& vel_cmd_prev, Vector4& wheel_cmd, const ros::Duration &dt);

    /*
     * publish odometry as topic & tf
     */
    void publish_odometry(const JointState<DOF_BASE>& basestate, const ros::Time& time);

    /*
     * publish internal driver state for debugging
     */
    void publish_driverstate(const omnibase_rt_driver::DataVec& data_vec, const ros::Time& time);

    void jac_forward(const Vector3 &X, Vector4 &W);
    void jac_inverse(const Vector4 &W, Vector3 &X);

    // private:
    // bool reset_odom_handler(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    static Vector3 mean(const std::vector<Vector3>& vec, size_t n);
    static Vector3 var(const std::vector<Vector3>& vec, size_t n, const Vector3& mean_val);

  private:
    // flags
    bool is_thread_stopped_;
    bool do_thread_stop_;
    bool wait_for_valid_reading_;
    bool is_first_iteration_;
    bool has_valid_state_;

    // configuation
    OmnibaseConfig config_;

    // communication
    omnibase_rt_driver::OmnibaseRTDriver driver_;

    // states
    JointState<DOF_BASE> js_curr_;    // current base state     
    JointState<DOF_BASE> js_prev_;    // previous base state

    VectorDOFBase vel_cmd_;     // commanded by write()
    VectorDOFBase vel_cmd_prev_; // previous filterd vel_cmd
    std::vector<VectorDOFBase> vel_hist_;   // velocity history
    size_t cnt;   // counter

    Vector4 wheel_pos_;       // wheel position read from driver
    Vector4 wheel_pos_prev_;  // wheel position in previous cycle
    Vector4 wheel_cmd_;       // wheel velocity command sent to the mobile base

    // time
    ros::Time time_start_;
    ros::Time time_cmd_write_;  // last time write command
    ros::Time time_odom_pub_;   // last time publish odometry
    ros::Time time_state_pub_;  // last time publish driver state 

    // config file path
    std::string config_file_path_;

    // protection jointstate
    std::mutex js_mutex_;

    // ros pub/srv
    ros::Publisher odom_pub_;
    ros::Publisher state_pub_;
    //  ros::ServiceServer reset_srv_;

    tf::TransformBroadcaster br;    // pub baseframe tf
  };

}

#endif