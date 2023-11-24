#include <tomm_hardware_real/arm/arm_interface.h>

#include <QElapsedTimer>
 
namespace tomm_hw
{
  ArmInterface::ArmInterface(const std::string& config_file_path, const std::string &int_pid_params_path) : 
    config_file_path_(config_file_path),
    int_pid_params_path_(int_pid_params_path),
    wait_for_valid_joint_pos_(true),
    is_thread_stopped_(true),
    do_thread_stop_(false),
    has_valid_state_(false),
    Kp_(VectorDOFArm::Zero()),
    Kd_(VectorDOFArm::Zero()),
    Ki_(VectorDOFArm::Zero())
  {
    if (TThread::isRtThread())
    {
      TThread::rtThread()->setPriority(8);
      TThread::rtThread()->setStackSize(16 * 1024 * 1024);
      TThread::rtThread()->setStackPreFaultSize(64 * 1024 * 1024);
    }
  }

  ArmInterface::~ArmInterface()
  {
    QElapsedTimer timer;
    timer.start();
    while ((!is_thread_stopped_) && (timer.elapsed() < 5000))
      usleep(5 * 1E5);
    if(is_thread_stopped_)
    {
      ROS_INFO("'%s' ArmInterface stopped", config_.ns.c_str());
      return;
    }

    if(!TThread::wait(500))
    {
      TThread::terminate();
      TThread::wait();
      ROS_WARN("'%s' ArmInterface forced stopped", config_.ns.c_str());
    }
  }

  bool ArmInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& hw_nh)
  {
    // parse the configuration
    if(!config_.load(config_file_path_))
    {
      ROS_ERROR("'%s' ArmInterface::init(): Failed to load config '%s'", config_.ns.c_str(), config_file_path_.c_str());
      return false;
    }
    ROS_WARN("ArmConfig:\n%s", config_.toString().c_str());

    // set inital to loaded
    js_init_ = config_.js_home;
    js_real_ = js_init_;
    js_cmd_ = js_init_;

    // set filters
    filter_delta_q_.setFilterParams(config_.butterworth_f_cutoff, config_.control_period);
    filter_q_real_.setFilterParams(config_.butterworth_f_cutoff, config_.control_period);
    filter_q_virt_.setFilterParams(config_.butterworth_f_cutoff, config_.control_period);
    filter_delta_v_.setFilterParams(config_.butterworth_f_cutoff, config_.control_period);
    filter_v_real_.setFilterParams(config_.butterworth_f_cutoff, config_.control_period);
    filter_v_virt_.setFilterParams(config_.butterworth_f_cutoff, config_.control_period);
    filter_v_comp_.setFilterParams(2.0, config_.control_period);

    // load internal control gains
    yaml::Parameters pid_params("pid_int");
    if (!pid_params.loadFile(int_pid_params_path_))
    {
      ROS_ERROR("'%s' ArmInterface::init(): Failed loading internal pid parameters file %s", config_.ns.c_str(), int_pid_params_path_.c_str());
      return false;
    }

    // store parameters
    pid_params.get("gains_d", Kd_);
    pid_params.get("gains_p", Kp_);
    pid_params.get("gains_i", Ki_);
    pid_params.get("lo_thresh", v_thresh_);

    ROS_WARN_STREAM("Kd_:" << Kd_.transpose());
    ROS_WARN_STREAM("Kp_:" << Kp_.transpose());
    ROS_WARN_STREAM("Ki_:" << Ki_.transpose());
    ROS_WARN_STREAM("v_thresh_:" << v_thresh_.transpose());    

    // finally, turn on the robot communication if required
    if(config_.is_real_robot)
    {
      ROS_INFO("Robot Arm '%s' is running REAL", config_.ns.c_str());
      QString ns = QString::fromStdString("tomm/arm_" + config_.ns);
      ROS_INFO_STREAM("Namespace=" << ns.toStdString());
      comm_interface_ = std::make_unique<ArmCommInterface>(ns, QString::fromStdString(config_file_path_));
      if(comm_interface_->error())
      {
        ROS_ERROR("'%s' ArmInterface::init(): ArmCommInterface initalization error", config_.ns.c_str());
        return false;
      }
    }
    else
    {
      ROS_INFO("Robot Arm '%s' is running SIM", config_.ns.c_str());
    }

    return true;
  }

  void ArmInterface::stop()
  {
    if(comm_interface_)
    {
      comm_interface_->setFinishMState();                                       // TODO if adding thread set stop flag here!
    }
    do_thread_stop_ = true;
  }

  void ArmInterface::run()
  {
    is_thread_stopped_ = false;
    wait_for_valid_joint_pos_ = true;
    is_first_iteration_ = true;
    has_valid_state_ = false;

    time_start_ = ros::Time::now();
    ros::Time time = time_start_;
    ros::Time time_prev = time_start_;
    ros::Duration dt;

    ROS_WARN_STREAM(config_.ns << " ArmInterface::run(): started");
    while(!do_thread_stop_)
    {
      time = ros::Time::now();
      dt = time - time_prev;
      time_prev = time;

      // real robot
      if(config_.is_real_robot)
      {
        // check if there's error in comm interface
        if(comm_interface_->error())
        {
          ROS_ERROR("'%s' ArmInterface::run(): Error stopping", config_.ns.c_str());
          stop();
          break; 
        }

        if(!updateReal(time, dt))
        {
          ROS_ERROR("'%s' ArmInterface::run(): Error stopping", config_.ns.c_str());
          stop();
          break;
        }
      }

      // sim robot
      if(!config_.is_real_robot)
      {
        if(!updateSim(time, dt))
        {
          ROS_ERROR("'%s' ArmInterface::run(): Error stopping", config_.ns.c_str());
          stop();
          break;
        }
      }

      // wait sample time
      TThread::usleep(config_.control_period * 1000 * 1000);
    }

    // thread is stopped, reset states
    is_thread_stopped_ = true;
    has_valid_state_ = false;
    ROS_WARN_STREAM(config_.ns << " ArmInterface::run(): stopped");
  }

  bool ArmInterface::updateSim(const ros::Time& time, const ros::Duration& dt)
  {
    // init in first iteration
    if(is_first_iteration_)
    {
      js_send_ = js_init_;
      js_send_prev_ = js_init_;
      js_cmd_ = js_init_;
      js_real_ = js_init_;
      is_first_iteration_ = false;
      has_valid_state_ = true;
    }

    // no pid in simulation
    js_send_ = js_cmd_;

    // check if command is valid
    if(!checkValidJointstateCmd(js_send_, js_send_prev_))
    {
      ROS_ERROR_STREAM("ArmInterface::updateSim() invalid" << 
        " q=" << js_cmd_.pos().transpose() << 
        " v=" << js_cmd_.vel().transpose());
      return false;
    }
    js_send_prev_ = js_send_;

    // sim robot
    js_real_ = js_send_;

    return true;
  }

  bool ArmInterface::updateReal(const ros::Time& time, const ros::Duration& dt)
  {
    // check if comm interface is ready
    ros::Time time_local = time;
    while(!comm_interface_->isReady() && (time_local - time_start_) < ros::Duration(5.0))
    {
      ROS_INFO_STREAM_THROTTLE(1.0, config_.ns << " ArmInterface::updateReal(): Wait for comm_interface_ is_ready");
      TThread::usleep(100 * 1000);
      time_local = ros::Time::now();
    }
    if(!comm_interface_->isReady())
    {
      ROS_ERROR_STREAM("ArmInterface::updateReal: ArmCommInterface not ready, arm=" << config_.ns);
      return false;
    }

    // check if robot returns valid pos
    if(wait_for_valid_joint_pos_)
    {
      bool valid = false;
      while(!do_thread_stop_ && !valid)
      {
        js_real_ = comm_interface_->getCurrentJointState();
        valid = checkValidInitalJointPos(js_real_.pos());
        TThread::usleep(100 * 1000); // 100 ms 
      }
      wait_for_valid_joint_pos_ = false;
    }

    // read the intial state and set everything to it
    js_real_ = comm_interface_->getCurrentJointState();

    // init if first iteration
    if(is_first_iteration_)
    {
      js_init_ = js_real_;
      js_send_ = js_init_;
      js_send_prev_ = js_init_;
      js_cmd_ = js_init_;
      is_first_iteration_ = false;
      has_valid_state_ = true;
    }

    // pull the real robot on commanded singal (such that real=cmd hold, hopefully...)
    VectorDOFArm qf_real = filter_q_real_.filter(js_real_.pos());
    VectorDOFArm qf_virt = filter_q_virt_.filter(js_cmd_.pos());

    VectorDOFArm Dq = qf_virt - qf_real;
    VectorDOFArm Dqf = filter_delta_q_.filter(Dq);

    VectorDOFArm qpf_real = filter_v_real_.filter(js_real_.vel());
    VectorDOFArm qpf_virt = filter_v_virt_.filter(js_cmd_.vel());

    VectorDOFArm Dqp = qpf_virt - js_real_.vel();

    VectorDOFArm v_comp = Kp_.cwiseProduct(Dq) - Kd_.cwiseProduct(qpf_real);
    VectorDOFArm vf_comp = filter_v_comp_.filter(v_comp);

    for (int i = 0; i < DOF_ARM; i++)
    {
      if (std::abs(v_comp(i)) < v_thresh_(i))
      {
        v_comp(i) = 0.0;
      }
    }

    VectorDOFArm v_des = qpf_virt + v_comp;

    js_send_.vel() = v_des;
    js_send_.pos() += v_des * config_.control_period;

    // check if valid js
    if(!checkValidJointstateCmd(js_send_, js_send_prev_))
    {
      ROS_ERROR_STREAM(config_.ns <<" ArmInterface::updateReal() invalid js send" << 
        "\n q=" << js_send_.pos().transpose() << 
        "\n v=" << js_send_.vel().transpose());
      return false;
    }
    js_send_prev_ = js_send_;

    // send to the real robot
    comm_interface_->setQd(js_send_.pos(), js_send_.vel());

    return true;
  }

  bool ArmInterface::read(JointState<DOF_ARM>& js, const ros::Time& time, const ros::Duration& dt)
  {
    // note: Here we are cheating (we are not returning real! but cmd)
    const std::lock_guard<std::mutex> lock(js_mutex_);
    js.pos() = js_cmd_.pos();
    js.vel() = js_cmd_.vel();
    return has_valid_state_;
  }

  bool ArmInterface::write(const JointState<DOF_ARM>& js, const ros::Time& time, const ros::Duration& dt)
  {
    const std::lock_guard<std::mutex> lock(js_mutex_);
    js_cmd_.pos() = js.pos();
    js_cmd_.vel() = js.vel();
    return has_valid_state_;
  }

  bool ArmInterface::checkValidJointstateCmd(
    const JointState<DOF_ARM>& js, const JointState<DOF_ARM>& js_prev)
  {
    for(int i=0; i < js.pos().size(); i++)
    {
      // check position
      if((std::isnan(js.pos()(i))) || (std::isinf(js.pos()(i))))
      {
        ROS_ERROR_STREAM(config_.ns << " Pos q[" << i << "] has NAN/INF");
        return false;
      }

      // check velo
      if((std::isnan(js.vel()(i))) || (std::isinf(js.vel()(i))))
      {
        ROS_ERROR_STREAM(config_.ns << " Vel v[" << i << "] has NAN/INF");
        return false;
      }

      // check velo limits
      if(std::abs(js.vel()(i)) > config_.v_max_limit)
      {
        ROS_ERROR_STREAM(config_.ns << " Vel q[" << i << "]=" << 
          std::abs(js.vel()(i)) << " > " << config_.v_max_limit);
        return false;
      }

      // check jump limits
      if(std::abs(js.pos()(i) - js_prev.pos()(i)) > config_.q_step_limit)
      {
        ROS_ERROR_STREAM(config_.ns << " Jump in detla_q[" << i << "]=" << 
          std::abs(js.pos()(i) - js_prev.pos()(i)) << " > " << config_.q_step_limit);
        return false;
      }
    }
    return true;
  }

  bool ArmInterface::checkValidInitalJointPos(const VectorDOFArm& q)
  {
    for(size_t i = 0; i < DOF_ARM; ++i)
    {
      if(std::abs(js_real_.pos()(i)) > 1e-3)
      {
        return true;
      }
    }
    return false;
  }
  
  bool ArmInterface::isRunning() const
  {
    return !is_thread_stopped_;
  }

  bool ArmInterface::hasValidState() const
  {
    return has_valid_state_;
  }

  const ArmConfig& ArmInterface::config() const
  {
    return config_;
  }

}