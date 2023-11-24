#include <tomm_hardware_real/tomm_hardware_real.h>

namespace tomm_hw
{
  TOMMHardwareReal::TOMMHardwareReal(
      const std::string &l_config_file_path,
      const std::string &r_config_file_path,
      const std::string &internal_pid_file_path,
      const std::string &base_config_file_path,
      bool verbose) : l_config_file_path_(l_config_file_path),
                      r_config_file_path_(r_config_file_path),
                      internal_pid_file_path_(internal_pid_file_path),
                      base_config_file_path_(base_config_file_path),
                      has_valid_connection_(false),
                      shut_down_(false),
                      verbose_(verbose)
  {
  }

  TOMMHardwareReal::~TOMMHardwareReal()
  {
    if (l_arm_interface_)
      l_arm_interface_->stop();
    if (r_arm_interface_)
      r_arm_interface_->stop();
    if (omnibase_interface_)
      omnibase_interface_->stop();
  }

  bool TOMMHardwareReal::init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh)
  {
    // ros connection
    shutdown_server_ = root_nh.advertiseService("shutdown", &TOMMHardwareReal::shutDownService, this);
    shutdown_client_ = root_nh.serviceClient<std_srvs::Empty>("/h1/shutdown");

    // setup the communication channels
    l_arm_interface_ = std::make_unique<ArmInterface>(l_config_file_path_, internal_pid_file_path_);
    if (!l_arm_interface_->init(root_nh, hw_nh))
    {
      ROS_ERROR("TOMMHardwareReal::init(): LEFT ArmInterface failed");
      return false;
    }
    r_arm_interface_ = std::make_unique<ArmInterface>(r_config_file_path_, internal_pid_file_path_);
    if (!r_arm_interface_->init(root_nh, hw_nh))
    {
      ROS_ERROR("TOMMHardwareReal::init(): RIGHT ArmInterface failed");
      return false;
    }
    omnibase_interface_ = std::make_unique<OmnibaseInterface>(base_config_file_path_);
    if (!omnibase_interface_->init(root_nh, hw_nh))
    {
      ROS_ERROR("TOMMHardwareReal::init(): OmnibaseInterface failed");
      return false;
    }
    ROS_WARN("Comm interfaces initialized.");

    // setup the hardware interfaces
    const ArmConfig &l_config = l_arm_interface_->config();
    const ArmConfig &r_config = r_arm_interface_->config();
    const OmnibaseConfig &base_config = omnibase_interface_->config();

    // default set arm js_, js_cmd to home pose
    l_js_ = l_config.js_home;
    l_js_cmd_ = l_config.js_home;
    r_js_ = r_config.js_home;
    r_js_cmd_ = r_config.js_home;

    // default set base js_, js_cmd to zero
    base_js_.setZero();
    base_js_cmd_.setZero();

    // register left arm into joint handle
    for (size_t i = 0; i < DOF_ARM; ++i)
    {
      // jointstate interface
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          l_config.joint_names[i], &l_js_.pos()[i], &l_js_.vel()[i], &l_js_.tau()[i]));
      // posvel command interface
      pvj_interface_.registerHandle(hardware_interface::PosVelJointHandle(
          js_interface_.getHandle(l_config.joint_names[i]), &l_js_cmd_.pos()[i], &l_js_cmd_.vel()[i]));
    }

    // register right arm into joint handle
    for (size_t i = 0; i < DOF_ARM; ++i)
    {
      // jointstate interface
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          r_config.joint_names[i], &r_js_.pos()[i], &r_js_.vel()[i], &r_js_.tau()[i]));
      // cmd jointposition interface
      pvj_interface_.registerHandle(hardware_interface::PosVelJointHandle(
          js_interface_.getHandle(r_config.joint_names[i]), &r_js_cmd_.pos()[i], &r_js_cmd_.vel()[i]));
    }

    // register omnibase into joint handle
    for (size_t i = 0; i < DOF_BASE; ++i)
    {
      // jointstate interface
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          base_config.joint_names[i], &base_js_.pos()[i], &base_js_.vel()[i], &base_js_.tau()[i]));

      // cmd jointposition interface
      pvj_interface_.registerHandle(hardware_interface::PosVelJointHandle(
          js_interface_.getHandle(base_config.joint_names[i]), &base_js_cmd_.pos()[i], &base_js_cmd_.vel()[i]));
    }

    Base::registerInterface(&js_interface_);
    Base::registerInterface(&pvj_interface_);

    has_valid_connection_ = false;
    return true;
  }

  bool TOMMHardwareReal::start(const ros::Time &time)
  {
    // start robot communication thread
    l_arm_interface_->start();
    r_arm_interface_->start();
    omnibase_interface_->start();

    ros::Time start_time = time;
    ros::Time cur_time = time;

    // wait for the inital robot state before starting the roscontrol loop
    ros::Rate rate(500);
    read(time, ros::Duration(0));
    while (!hasValidConnection() && (cur_time - start_time) < ros::Duration(5.0))
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "TOMMHardwareReal::start(): Wait valid " << (cur_time - start_time).toSec() << " sec");
      cur_time = ros::Time::now();
      read(time, ros::Duration(0));
      rate.sleep();
    }
    if (!hasValidConnection())
    {
      ROS_ERROR("TOMMHardwareReal::start(): Failed to get valid robot connection");
      return false;
    }

    // set initial arm cmd to current
    l_js_cmd_ = l_js_;
    r_js_cmd_ = r_js_;

    // set initial base cmd to zero
    base_js_cmd_.setZero();

    return true;
  }

  bool TOMMHardwareReal::hasValidConnection() const
  {
    return has_valid_connection_;
  }

  void TOMMHardwareReal::read(const ros::Time &time, const ros::Duration &dt)
  {
    // js_ values only valid if has_valid_connection_=true
    bool l_valid = l_arm_interface_->read(l_js_, time, dt);
    bool r_valid = r_arm_interface_->read(r_js_, time, dt);
    bool base_valid = omnibase_interface_->read(base_js_, time, dt);

    has_valid_connection_ = l_valid && r_valid && base_valid;

    if (verbose_)
    {
      ROS_WARN_STREAM_THROTTLE(0.5, "l_cur=" << l_js_.pos().transpose());
      ROS_WARN_STREAM_THROTTLE(0.5, "r_cur=" << r_js_.pos().transpose());
      ROS_WARN_STREAM_THROTTLE(0.5, "base_cur_pos_=" << base_js_.pos().transpose());
      ROS_WARN_STREAM_THROTTLE(0.5, "base_cur_vel_=" << base_js_.vel().transpose() << "\n");
    }
  }

  void TOMMHardwareReal::write(const ros::Time &time, const ros::Duration &dt)
  {
    bool l_valid = l_arm_interface_->write(l_js_cmd_, time, dt);
    bool r_valid = r_arm_interface_->write(r_js_cmd_, time, dt);
    bool base_valid = omnibase_interface_->write(base_js_cmd_, time, dt);

    has_valid_connection_ = l_valid && r_valid && base_valid;

    if (verbose_)
    {
      ROS_INFO_STREAM_THROTTLE(0.5, "l_cmd=" << l_js_cmd_.pos().transpose());
      ROS_INFO_STREAM_THROTTLE(0.5, "r_cmd=" << r_js_cmd_.pos().transpose());
      ROS_INFO_STREAM_THROTTLE(0.5, "base_vel_cmd=" << base_js_cmd_.vel().transpose() << "\n");
    }
  }

  void TOMMHardwareReal::writeZero(const ros::Time &time, const ros::Duration &dt)
  {
    ROS_WARN_STREAM("TOMMHardwareReal::writeZero(): Stop all commands");
    // set arm command to current position with zero velocity
    l_js_cmd_.pos() = l_js_.pos();
    l_js_cmd_.vel().setZero();
    l_js_cmd_.acc().setZero();

    r_js_cmd_.pos() = r_js_.pos();
    r_js_cmd_.vel().setZero();
    r_js_cmd_.acc().setZero();

    // set base velocity command to zero
    base_js_cmd_.setZero();

    // write one last time
    write(time, dt);

    if (verbose_)
    {
      ROS_INFO_STREAM_THROTTLE(0.5, "l_cmd=" << l_js_cmd_.pos().transpose());
      ROS_INFO_STREAM_THROTTLE(0.5, "r_cmd=" << r_js_cmd_.pos().transpose());
      ROS_INFO_STREAM_THROTTLE(0.5, "base_vel_cmd=" << base_js_cmd_.vel().transpose() << "\n");
    }
  }

  bool TOMMHardwareReal::shutDownService(std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &res)
  {
    ROS_ERROR_STREAM("Shutting down by request");
    shut_down_ = true;
    return true;
  }

  bool TOMMHardwareReal::shutDown() const
  {
    return shut_down_;
  }

  void TOMMHardwareReal::requestH1ShutDown()
  {
    std_srvs::Empty srv;
    if (shutdown_client_.call(srv))
    {
      ROS_INFO("Requested H1 to shutdown");
    }
    else
    {
      ROS_ERROR("Failed to call service /h1/shutdown");
    }
  }
}