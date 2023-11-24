#include <tomm_hardware_real/omnibase/omnibase_interface.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <tomm_hardware_real/utilities/omnibase_util.h>
#include <tomm_hardware_real/OmniBaseState.h>

#include <QElapsedTimer>

#define HIST_SIZE 50

namespace tomm_hw
{
  OmnibaseInterface::OmnibaseInterface(const std::string &config_file_path) : config_file_path_(config_file_path),
                                                                              driver_(),
                                                                              is_thread_stopped_(true),
                                                                              do_thread_stop_(false),
                                                                              has_valid_state_(false),
                                                                              is_first_iteration_(true)
  {
    if (TThread::isRtThread())
    {
      TThread::rtThread()->setPriority(95);
      TThread::rtThread()->setStackSize(16 * 1024 * 1024);
      TThread::rtThread()->setStackPreFaultSize(64 * 1024 * 1024);
    }
  }

  OmnibaseInterface::~OmnibaseInterface()
  {
    QElapsedTimer timer;
    timer.start();
    while ((!is_thread_stopped_) && (timer.elapsed() < 5000))
      usleep(5 * 1E5);
    if (is_thread_stopped_)
    {
      ROS_INFO(" OmnibaseInterface stopped");
      return;
    }

    if (!TThread::wait(500))
    {
      TThread::terminate();
      TThread::wait();
      ROS_WARN("' OmnibaseInterface forced stopped");
    }
  }

  bool OmnibaseInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh)
  {
    // parse the configuration
    if (!config_.load(config_file_path_))
    {
      ROS_ERROR("RobotArmCommInterface::init(): Failed to load config '%s'", config_file_path_.c_str());
      return false;
    }
    ROS_WARN("OmnibaseConfig:\n%s", config_.toString().c_str());

    // set initial base state to zero
    js_curr_.setZero();
    js_prev_.setZero();

    // set initial cmd to zero
    vel_cmd_.setZero();
    vel_cmd_prev_.setZero();
    wheel_cmd_.setZero();

    // set initial wheel position to zero
    wheel_pos_.setZero();
    wheel_pos_prev_.setZero();

    vel_hist_.resize(HIST_SIZE);

    // setup ros pub
    odom_pub_ = hw_nh.advertise<nav_msgs::Odometry>("odom", 10);
    state_pub_ = hw_nh.advertise<tomm_hardware_real::OmniBaseState>(config_.ns + "/omnibase_state", 10);
    // reset_srv_ = hw_nh.advertiseService(ns + "/reset_odometry", &OmnibaseDriver::reset_odom_handler, this);

    // init driver if running real
    if (config_.is_real_robot)
    {
      ROS_INFO("Mobile base is running REAL");
      driver_.set_interface(config_.interface);
      if (!driver_.init())
      {
        ROS_ERROR("OmnibaseInterface::init(): Omnibase driver initalization error");
        return false;
      }
    }
    else
    {
      ROS_INFO("Mobile base is running SIM");
    }

    return true;
  }

  void OmnibaseInterface::stop()
  {
    if (config_.is_real_robot)
      driver_.stop_request();
    do_thread_stop_ = true;
  }

  void OmnibaseInterface::run()
  {
    is_thread_stopped_ = false;
    has_valid_state_ = false;

    time_start_ = ros::Time::now();
    ros::Time time = time_start_;
    ros::Time time_prev = time_start_;
    time_cmd_write_ = time_start_;
    ros::Duration dt;

    ROS_WARN_STREAM(" OmnibaseInterface::run(): started");
    while (!do_thread_stop_)
    {
      time = ros::Time::now();
      dt = time - time_prev;
      time_prev = time;

      // real robot
      if (config_.is_real_robot)
      {
        if (!updateReal(time, dt))
        {
          ROS_ERROR("OmnibaseInterface::updateReal(): Error! stopping");
          stop();
          break;
        }
      }

      // sim robot
      if (!config_.is_real_robot)
      {
        if (!updateSim(time, dt))
        {
          ROS_ERROR("OmnibaseInterface::updateSim(): Error! stopping");
          stop();
          break;
        }
      }

      // wait sample time
      TThread::usleep(2 * 1000); // 2ms
    }

    // thread is stopped, reset states
    is_thread_stopped_ = true;
    has_valid_state_ = false;
    ROS_WARN_STREAM("OmnibaseInterface::run(): stopped");
  }

  bool OmnibaseInterface::updateSim(const ros::Time &time, const ros::Duration &dt)
  {
    if (is_first_iteration_)
    {
      has_valid_state_ = true;
      is_first_iteration_ = false;
    }

    // check watchdog
    double write_dt = (time - time_cmd_write_).toSec();
    if (write_dt > 0.4)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "watchdog timer hit! dt = " << write_dt);
      vel_cmd_.setZero();
      vel_cmd_prev_.setZero();
    }

    // check if command is valid
    checkValidCmd(vel_cmd_);

    // save command
    vel_cmd_prev_ = vel_cmd_;

    // sim odometry
    simulate_odometry(vel_cmd_, js_curr_, dt);
    publish_odometry(js_curr_, time);

    // save (sim) write time
    time_cmd_write_ = time;

    return true;
  }

  bool OmnibaseInterface::updateReal(const ros::Time &time, const ros::Duration &dt)
  {
    ros::Time time_local = time;

    // start driver
    driver_.start_request();
    // wait for driver to update
    while (!driver_.is_ready_read() && (time_local - time_start_) < ros::Duration(5.0))
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "OmnibaseInterface::updateReal(): Wait for omnibase rt_driver is_ready");
      TThread::usleep(10 * 1000);
      time_local = ros::Time::now();
    }
    if (!driver_.is_ready_read())
    {
      ROS_ERROR_STREAM("OmnibaseInterface::updateReal(): omnibase rt_driver not ready");
      return false;
    }

    // driver data
    omnibase_rt_driver::DataVec data_vec;

    // now there's valid state
    if (is_first_iteration_)
    {
      has_valid_state_ = true;
      is_first_iteration_ = false;

      // save initial driver position
      driver_.read(data_vec);
      if (data_vec.size() != NUM_WHEELS)
      {
        ROS_ERROR_STREAM("OmnibaseInterface::updateReal(): read wrong data size");
        return false;
      }
      for (size_t i{0}; i < NUM_WHEELS; ++i)
      {
        wheel_pos_prev_[i] = data_vec[i].position;
      }
    }

    // check watchdog
    double write_dt = (time - time_cmd_write_).toSec();
    if (write_dt > 0.4)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "watchdog timer hit! dt = " << write_dt);
      vel_cmd_.setZero();
      vel_cmd_prev_.setZero();
    }

    // check if command is valid
    checkValidCmd(vel_cmd_);

    // read current driver data;
    driver_.read(data_vec);
    if (data_vec.size() != NUM_WHEELS)
    {
      ROS_ERROR_STREAM("OmnibaseInterface::updateReal(): read wrong data size");
      return false;
    }

    // update odometry
    update_odometry(data_vec, js_curr_, dt);
    publish_odometry(js_curr_, time);

    // publish internal driver state
    publish_driverstate(data_vec, time);

    // update wheel cmd
    update_driver(vel_cmd_, vel_cmd_prev_, wheel_cmd_, dt);

    omnibase_rt_driver::VelocityVec send_cmd;
    for (size_t i = 0; i < NUM_WHEELS; ++i)
    {
      send_cmd[i].velocity = wheel_cmd_[i];
    }
    // send to the real robot
    driver_.write(send_cmd);

    return true;
  }

  bool OmnibaseInterface::read(JointState<DOF_BASE> &jointstate, const ros::Time &time, const ros::Duration &dt)
  {
    // return the base velocity updated by odometry
    const std::lock_guard<std::mutex> lock(js_mutex_);
    jointstate.pos() = js_curr_.pos();
    jointstate.vel() = js_curr_.vel();
    return has_valid_state_;
  }

  bool OmnibaseInterface::write(const JointState<DOF_BASE> &jointstate, const ros::Time &time, const ros::Duration &dt)
  {
    // if connection not established, don't write value
    if (!has_valid_state_)
      return false;

    const std::lock_guard<std::mutex> lock(js_mutex_);
    vel_cmd_ = jointstate.vel();
    time_cmd_write_ = time; // save cmd input time
    return has_valid_state_;
  }

  void OmnibaseInterface::checkValidCmd(
      VectorDOFBase &vel_cmd)
  {
    for (int i = 0; i < DOF_BASE; i++)
    {
      // check velo
      if ((std::isnan(vel_cmd(i))) || (std::isinf(vel_cmd(i))))
      {
        ROS_ERROR_STREAM("Vel v[" << i << "] has NAN/INF, set cmd to ZERO");
        vel_cmd(i) = 0;
      }
    }
  }

  void OmnibaseInterface::update_odometry(
      const omnibase_rt_driver::DataVec &data_vec, JointState<DOF_BASE> &basestate, const ros::Duration &dt)
  {
    for (size_t i = 0; i < data_vec.size(); ++i)
    {
      wheel_pos_[i] = data_vec[i].position;
    }
    // convert encoder difference to meters, convert meters to velocity
    Vector4 W_vel;
    for (int i = 0; i < W_vel.size(); ++i)
    {
      int ticks = wheel_pos_[i] - wheel_pos_prev_[i];
      W_vel[i] = ticks * (1.0 / (ODOMETRY_CONSTANT * ODOMETRY_CORRECTION));
    }
    wheel_pos_prev_ = wheel_pos_;

    // convert to cartesian velocity
    jac_inverse(W_vel, basestate.vel());

    basestate.vel() = -basestate.vel();

    // update pose
    double th = basestate.pos()[2]; // + basestate.vel()[2];
    basestate.pos()[0] += basestate.vel()[0] * cos(th) - basestate.vel()[1] * sin(th);
    basestate.pos()[1] += basestate.vel()[0] * sin(th) + basestate.vel()[1] * cos(th);
    basestate.pos()[2] += basestate.vel()[2];

    basestate.vel()[0] /= dt.toSec();
    basestate.vel()[1] /= dt.toSec();
    basestate.vel()[2] /= dt.toSec();

    vel_hist_[cnt % HIST_SIZE] = basestate.vel();
    cnt++;
  }

  void OmnibaseInterface::simulate_odometry(
      const Vector3 &vel_cmd, JointState<DOF_BASE> &basestate, const ros::Duration &dt)
  {
    // update velocity
    basestate.vel() = vel_cmd;

    // update pose
    double th = basestate.pos()[2]; // + basestate.vel()[2];
    basestate.pos()[0] += (basestate.vel()[0] * cos(th) - basestate.vel()[1] * sin(th)) * dt.toSec();
    basestate.pos()[1] += (basestate.vel()[0] * sin(th) + basestate.vel()[1] * cos(th)) * dt.toSec();
    basestate.pos()[2] += basestate.vel()[2] * dt.toSec();

    vel_hist_[cnt % HIST_SIZE] = basestate.vel();
    cnt++;
  }

  void OmnibaseInterface::publish_odometry(const JointState<DOF_BASE> &basestate, const ros::Time &time)
  {
    if ((time - time_odom_pub_).toSec() > config_.odom_pub_dt)
    {
      time_odom_pub_ = time;

      // comput mean, variance
      size_t n = std::min(int(cnt), HIST_SIZE);
      Vector3 vel_mean = mean(vel_hist_, n);
      Vector3 vel_var = var(vel_hist_, n, vel_mean);
      cnt = 0;

      tf::Quaternion q;
      q.setRPY(0, 0, basestate.pos()[2]);

      // publish tf
      if (config_.pub_tf)
      {
        tf::Transform pose(q, tf::Point(basestate.pos()[0], basestate.pos()[1], 0.0));
        br.sendTransform(tf::StampedTransform(pose, time, config_.frame_id, config_.child_frame_id));
        // ROS_ERROR_STREAM_THROTTLE(0.5, "frame_id: " << config_.frame_id);
        // ROS_ERROR_STREAM_THROTTLE(0.5, "child_frame_id: " << config_.child_frame_id);
      }

      // publish odometry msg
      if (config_.pub_odom_msg)
      {
        nav_msgs::Odometry msg;
        msg.header.stamp = time;
        msg.header.frame_id = config_.frame_id;      // position wrt odom
        msg.child_frame_id = config_.child_frame_id; // velocity wrt robot base
        // position
        msg.pose.pose.position.x = basestate.pos()[0];
        msg.pose.pose.position.y = basestate.pos()[1];
        msg.pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg(q, msg.pose.pose.orientation);
        // twist
        msg.twist.twist.linear.x = basestate.vel()[0];
        msg.twist.twist.linear.y = basestate.vel()[1];
        msg.twist.twist.linear.z = 0.0;
        msg.twist.twist.angular.x = 0.0;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = basestate.vel()[2];
        // position covariance matrix (guess)
        msg.pose.covariance.fill(0.0);
        /*msg.pose.covariance[0] = 0.1*0.1;   // 1cm
        msg.pose.covariance[6] = 0.1*0.1;   // 1cm
        msg.pose.covariance[35] = 0.1*0.1;*/
        // 0.1 rad
        // twist covarinace matrix
        msg.twist.covariance.fill(0.0);
        /*msg.twist.covariance[0] = Xp_var[0];
        msg.twist.covariance[6] = Xp_var[1];
        msg.twist.covariance[35] = Xp_var[0];*/
        odom_pub_.publish(msg);
      }
    }
  }

  bool OmnibaseInterface::isRunning() const
  {
    return !is_thread_stopped_;
  }

  bool OmnibaseInterface::hasValidState() const
  {
    return has_valid_state_;
  }

  const OmnibaseConfig &OmnibaseInterface::config() const
  {
    return config_;
  }

  void OmnibaseInterface::update_driver(
      const Vector3 &vel_cmd, Vector3 &vel_cmd_prev, Vector4 &wheel_cmd, const ros::Duration &dt)
  {
    // config values
    double radius = config_.radius;
    double cmd_acc_limit = config_.cmd_acc_limit;
    double cmd_vel_limit = config_.cmd_vel_limit;
    double cart_limit = config_.cart_limit;
    double wheel_limit = config_.wheel_limit;

    double corr_wheels, corr_cart, corr;

    Vector3 filtered_cmd;
    Vector3 acc = -vel_cmd - vel_cmd_prev; // note: reverse the sign of vel_cmd
    Vector3 fac_rot;
    fac_rot << 1.0, 1.0, 1.0 / radius;

    Vector4 Wp;

    // smooth the acceleration
    for (size_t i = 0; i < DOF_BASE; ++i)
    {
      acc[i] = LIMIT(acc[i], cmd_acc_limit * fac_rot[i] * fac_rot[i] / dt.toSec());
      filtered_cmd[i] = vel_cmd_prev[i] + acc[i];
      filtered_cmd[i] = LIMIT(filtered_cmd[i], cmd_vel_limit * fac_rot[i]);
    }
    vel_cmd_prev = filtered_cmd;

    // convert to wheel velocites
    jac_forward(filtered_cmd, Wp);

    // check velo limits
    // cartesian limit: add linear and angular parts
    corr_cart = cart_limit / filtered_cmd.norm();
    // wheel limit: for one wheel, x,y and a always add up
    corr_wheels = wheel_limit / (fabs(filtered_cmd[0]) + fabs(filtered_cmd[1]) + fabs(filtered_cmd[2]));
    // get limiting factor as min(1, corr_cart, corr_wheels)
    corr = (1 < corr_cart) ? 1 : ((corr_cart < corr_wheels) ? corr_cart : corr_wheels);

    wheel_cmd = corr * DRIVE_CONSTANT * Wp;
  }

  Vector3 OmnibaseInterface::mean(const std::vector<Vector3> &vec, size_t n)
  {
    Vector3 mean_val{Vector3::Zero()};

    if (n <= 0)
      return mean_val;

    for (size_t i{0}; i < n; ++i)
    {
      mean_val += vec[i];
    }
    mean_val /= double(n);

    return mean_val;
  }

  void OmnibaseInterface::jac_forward(const Vector3 &Xp, Vector4 &Wp)
  {
    // computing:
    //   out = (C*J_fwd) * in
    // with:
    //   J_fwd = [1 -1 -alpha;
    //            1  1  alpha;
    //            1  1 -alpha;
    //            1 -1  alpha]
    //   C     = [0  0  0  1;
    //            0  0 -1  0;
    //            0  1  0  0;
    //           -1  0  0  0 ]

    static double alpha = 0.39225 + 0.303495;
    static Eigen::Matrix<double, 4, 3> C_J_fwd;
    C_J_fwd << 1, -1, alpha,
        -1, -1, alpha,
        1, 1, alpha,
        -1, 1, alpha;

    Wp = C_J_fwd * Xp;
  }

  void OmnibaseInterface::jac_inverse(const Vector4 &Wp, Vector3 &Xp)
  {
    // computing:
    //   out = (J_inv*C^-1) * in
    // with:
    //   J_fwd = 1/4*[1 -1 -alpha;
    //                1  1  alpha;
    //                1  1 -alpha;
    //                1 -1  alpha]
    //   C = [0  0  0  1;
    //        0  0 -1  0;
    //        0  1  0  0;
    //       -1  0  0  0 ]

    static double alpha = 0.39225 + 0.303495;
    static Eigen::Matrix<double, 3, 4> C_J_inv;
    C_J_inv << 0.25, -0.25, 0.25, -0.25,
        -0.25, -0.25, 0.25, 0.25,
        0.25 / alpha, 0.25 / alpha, 0.25 / alpha, 0.25 / alpha;

    Xp = C_J_inv * Wp;
  }

  Vector3 OmnibaseInterface::var(const std::vector<Vector3> &vec, size_t n, const Vector3 &mean_val)
  {
    Vector3 var_val{Vector3::Zero()};
    Vector3 tmp{Vector3::Zero()};

    if (n <= 0)
      return mean_val;

    for (size_t i{0}; i < n; ++i)
    {
      tmp = vec[i] - mean_val;
      var_val[0] += tmp[0] * tmp[0];
      var_val[1] += tmp[1] * tmp[1];
      var_val[2] += tmp[2] * tmp[2];
    }
    var_val /= double(n - 1);

    return var_val;
  }

  void OmnibaseInterface::publish_driverstate(
      const omnibase_rt_driver::DataVec &data_vec, const ros::Time &time)
  {
    if ((time - time_state_pub_).toSec() > config_.state_pub_dt)
    {
      time_state_pub_ = time;

      tomm_hardware_real::OmniBaseState msg;
      msg.state.resize(NUMBER_OF_SLAVES);
      for (int i = 0; i < NUMBER_OF_SLAVES; ++i)
      {
        msg.state[i].position = data_vec[i].position;
        msg.state[i].velocity = data_vec[i].velocity;
        msg.state[i].digital_inputs = data_vec[i].digital_inputs;
        msg.state[i].status = data_vec[i].status;
      }
      state_pub_.publish(msg);
    }
  }

}