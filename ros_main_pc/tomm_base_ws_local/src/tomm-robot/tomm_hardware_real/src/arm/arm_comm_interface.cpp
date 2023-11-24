#include <ros/ros.h>

#include <tomm_hardware_real/arm/arm_comm_interface.h>
#include <yaml_parameters/yaml_parameters.h>

#include <QElapsedTimer>
#include <QApplication>
#include <QFileInfo>
#include <QSettings>
#include <unistd.h>

#include <QTextStream>
#include <QFile>
#include <QDataStream>

#define COMM_INTF_BEFORE_TRAJ_FOLLOW_WAIT_CYCLES 250 /* 2 sec @ 8 ms cycle time */

namespace tomm_hw
{

  const double ArmCommInterface::COMM_PERIOD = 0.008;
  const double ArmCommInterface::COMM_PERIOD_2 = 0.004;
  const double ArmCommInterface::MULT_jointstate = 1.0E8; // 10000.0;
  const double ArmCommInterface::MULT_time = 1000000.0;
  const double ArmCommInterface::MULT_blend = 1000.0;

  ArmCommInterface::ArmCommInterface(
      const QString &ns,
      const QString &pcIpAddr,
      const quint16 trajPort,
      const QString &robotIpAddr,
      const quint16 robotStatePort,
      QObject *parent) : TThread(RtThreads::Thread::RtThreadMode, "commInterface"),
                         m_ns(ns),
                         m_pcIp(pcIpAddr),
                         m_trajPort(trajPort),
                         m_robotIp(robotIpAddr),
                         m_robotStatePort(robotStatePort),
                         m_robotVersion(RobotStateClient::VERSION_1V8),
                         m_serverTraj(0),
                         m_socketTraj(0),
                         m_error(false),
                         m_stopThread(false),
                         m_stoppedThread(true),
                         m_mstate(STATE_INIT),
                         m_qStart(VectorDOFArm::Zero()),
                         m_qd(VectorDOFArm::Zero()),
                         m_qpd(VectorDOFArm::Zero()),
                         m_currentCycle(0),
                         m_waypointId(0),
                         m_robotStateClient(NULL),
                         m_ready(false),
                         m_motionCommand(POSITION_INTF),
                         m_pend_mstate(STATE_IDLE),
                         m_mstatePendFlag(false),
                         m_mWaitCnt(0),
                         m_cnt(0)
  {
    TThread::start();

    if (TThread::isRtThread())
    {
      qDebug("ArmCommInterface: Using RtThread");
      TThread::rtThread()->setPriority(7);
      TThread::rtThread()->setStackSize(16 * 1024 * 1024);
      TThread::rtThread()->setStackPreFaultSize(64 * 1024 * 1024);
    }
    else
    {
      qDebug("ArmCommInterface: Using QThread");
    }

    m_setScriptState = m_node.serviceClient<ur_script_manager::setScriptManagerState>("setScriptManagerState");
  }

  ArmCommInterface::ArmCommInterface(const QString &ns, const QString &configFilePath, RobotStateClient *extRobotStateClient) : TThread(RtThreads::Thread::RtThreadMode, "commInterface"),
                                                                                                                                m_ns(ns),
                                                                                                                                m_robotVersion(RobotStateClient::VERSION_1V8),
                                                                                                                                m_serverTraj(0),
                                                                                                                                m_socketTraj(0),
                                                                                                                                m_error(false),
                                                                                                                                m_stopThread(false),
                                                                                                                                m_stoppedThread(true),
                                                                                                                                m_mstate(STATE_INIT),
                                                                                                                                m_qStart(VectorDOFArm::Zero()),
                                                                                                                                m_qd(VectorDOFArm::Zero()),
                                                                                                                                m_qpd(VectorDOFArm::Zero()),
                                                                                                                                m_currentCycle(0),
                                                                                                                                m_waypointId(0),
                                                                                                                                m_robotStateClient(extRobotStateClient),
                                                                                                                                m_ready(false),
                                                                                                                                m_motionCommand(POSITION_INTF),
                                                                                                                                m_pend_mstate(STATE_IDLE),
                                                                                                                                m_mstatePendFlag(false),
                                                                                                                                m_mWaitCnt(0),
                                                                                                                                m_cnt(0)
  {

    if (!parseConfig(configFilePath))
    {
      return;
    }

    TThread::start();

    if (TThread::isRtThread())
    {
      qDebug("ArmCommInterface: Using RtThread");
    }
    else
    {
      qDebug("ArmCommInterface: Using QThread");
    }
    m_setScriptState = m_node.serviceClient<ur_script_manager::setScriptManagerState>(m_ns.toStdString() + "/setScriptManagerState");

    // We need to control if the robotStateClient (rsc) is internally created
    // or externally provided
    // this flag will be used in the destructor, if the rsc is internally created
    // then we need to delete it, otherwise it should be deleted externally.
    if (extRobotStateClient != NULL)
    {
      m_extRobotSateClientFlag = true;
    }
    else
    {
      m_extRobotSateClientFlag = false;
    }
  }

  ArmCommInterface::~ArmCommInterface()
  {
    setFinishMState();

    QElapsedTimer timer;
    timer.start();

    // Wait until stoppedThread changes or the limit time has been reached (5secs)
    while ((!m_stoppedThread) && (timer.elapsed() < 5000))
    {
      ::usleep(5 * 1E5);
    }

    if (m_stoppedThread)
    {
      cleanup();
      qDebug("ArmCommInterface class destroyed.");
      return;
    }

    qWarning("WARNING: Waiting for thread of ArmCommInterface class to finish ...");

    // if the thread has not been stopped
    if (!TThread::wait(500))
    {
      TThread::terminate();
      TThread::wait();
      qDebug() << "ArmCommInterface::~ArmCommInterface: Thread forced to end!!!";
    }

    cleanup();
    qDebug("ArmCommInterface class destroyed.");
  }

  bool ArmCommInterface::error()
  {
    return m_error;
  }
  const QString &ArmCommInterface::errorString()
  {
    return m_errorString;
  }

  JointState<DOF_ARM> ArmCommInterface::getCurrentJointState()
  {
    JointState<DOF_ARM> jS;

    m_mutexJointState.lock();
    jS = m_jointState;
    m_mutexJointState.unlock();

    return jS;
  }

  void ArmCommInterface::setQd(const VectorDOFArm &qTarget, const VectorDOFArm &qpTarget)
  {
    m_mutex.lock();
    m_qd = qTarget;
    m_qpd = qpTarget;
    m_mutex.unlock();
  }

  void ArmCommInterface::setIdleMState()
  {
    m_mutex.lock();
    if (m_mstate == STATE_TRAJ_FOLLOW)
    {
      m_pend_mstate = STATE_IDLE;
      m_mstatePendFlag = true;
    }
    m_mutex.unlock();
  }

  void ArmCommInterface::setFinishMState()
  {
    m_mutex.lock();

    if (m_mstate == STATE_TRAJ_FOLLOW)
    {
      m_pend_mstate = STATE_FINISH;
      m_mstatePendFlag = true;
    }
    m_mutex.unlock();
  }
  ArmCommInterface::State ArmCommInterface::getMState()
  {
    State s;
    m_mutex.lock();
    s = m_mstate;
    m_mutex.unlock();
    return s;
  }

  bool ArmCommInterface::robotStateIsReady() const
  {
    return m_robotStateClient->isReady();
  }

  bool ArmCommInterface::isReady() const
  {
    return m_ready;
  }

  const ArmCommInterface::MotionCommand ArmCommInterface::getMotionIntf() const
  {
    return m_motionCommand;
  }
  void ArmCommInterface::setMotionIntf(const MotionCommand intf)
  {
    m_motionCommand = intf;
  }

  bool ArmCommInterface::init()
  {
    // wait until service available
    if (!ros::service::waitForService(m_ns.toStdString() + "/setScriptManagerState", 1.0))
    {
      m_error = true;
      m_errorString = "ArmCommInterface::waitForService(): Failed to contact script manager.";
      qDebug("%s", m_errorString.toLatin1().data());
      return false;
    }

    // stop all traj script threads
    bool ret = enableTrajScript(POSITION_INTF, false);
    TThread::usleep(500 * 1000);

    ret = enableTrajScript(VELOCITY_INTF, false) && ret;
    TThread::usleep(500 * 1000);

    if (!ret)
    {
      m_error = true;
      m_errorString = "ArmCommInterface::init(): Failed to contact script manager.";
      qDebug("%s", m_errorString.toLatin1().data());
      return false;
    }

    // Start the client to get the robot state from port 30003
    if (!m_extRobotSateClientFlag)
    {
      // If the RSC is not externally, we need to creat it.
      m_robotStateClient = new RobotStateClient(m_robotIp,
                                                m_robotVersion,
                                                m_robotStatePort);
      if (m_robotStateClient->error())
      {
        m_error = true;
        m_errorString = "ArmCommInterface::init(): RobotStateClient: " + m_robotStateClient->errorString();
        qDebug() << m_errorString;
        return false;
      }
    }

    return true;
  }

  void ArmCommInterface::run()
  {

    m_stoppedThread = false;

    if (!init())
    {
      m_stoppedThread = true;
      qDebug() << "ArmCommInterface::run(): !init-->Thread exited";
      return;
    }

    while (!m_stopThread)
    {
      QElapsedTimer tic;

      tic.start();

      m_robotStateClient->update();

      // Try to read the current JointState from UR controller
      m_mutexJointState.lock();
      m_jointState = m_robotStateClient->getJointState();
      m_mutexJointState.unlock();

      //        qDebug("ArmCommInterface: After update state");
      //        qDebug()<<"Joints: "<<m_jointState.toString();

      // calculate the next state of the state machine
      updateStateMachine();

      // executes the correspondign functions to the current state of the state machine
      executeMState();

      double elapsedTime = tic.nsecsElapsed() * 1E-9;
      double delay = (COMM_PERIOD - elapsedTime) * 1E6;

      //        ROS_INFO_STREAM("E time: "<<elapsedTime);

      if (m_mstate == STATE_TRAJ_FOLLOW && elapsedTime > COMM_PERIOD)
      {
        ROS_WARN_STREAM("Communication loop period exceeds the limit: " << elapsedTime);
        continue;
      }

      if (TThread::isRtThread())
      {
        //            qDebug("RtThread sleep");
        TThread::usleep(COMM_PERIOD * 1e6);
      }
      else
      {
        //            qDebug("Normal sleep");
        if (delay <= 0)
        {
          TThread::usleep(10 * 1000);
          continue;
        }
        TThread::usleep((unsigned long)delay);
      }
    }

    m_stoppedThread = true;
    qDebug("ArmCommInterface::run(): Thread exited");
  }

  void ArmCommInterface::stop()
  {
    if (m_stoppedThread)
    {
      return;
    }

    m_stopThread = true;
    qDebug() << "ArmCommInterface::stop(): m_stopThread=" << m_stopThread;
  }

  void ArmCommInterface::cleanup()
  {
    if (!m_extRobotSateClientFlag)
    {
      // If the ext RSC then it has been created internally, therfore it needs to be
      // deleted!
      if (m_robotStateClient != NULL)
      {
        delete m_robotStateClient;
        m_robotStateClient = NULL;
      }
    }
  }

  void ArmCommInterface::updateStateMachine()
  {
    // Get the JointState from the controller (port 50001)

    if (m_mstate == STATE_INIT)
    {
      //        ROS_INFO_STREAM("STATE_INIT up");

      if (m_robotStateClient != NULL)
      {
        if (m_robotStateClient->isReady())
        {
          qDebug("STATE_INIT: Robot state client is ready.");
          m_mutex.lock();
          m_mstate = STATE_IDLE;
          m_mutex.unlock();
        }
      }
    }
    else if (m_mstate == STATE_IDLE)
    {
      ROS_INFO_STREAM("STATE_IDLE up");
      m_mutex.lock();
      m_mstate = STATE_BEFORE_TRAJ_FOLLOW_WAIT;
      m_mutex.unlock();
    }
    else if (m_mstate == STATE_BEFORE_TRAJ_FOLLOW_WAIT)
    {
      if (m_mWaitCnt > COMM_INTF_BEFORE_TRAJ_FOLLOW_WAIT_CYCLES)
      {
        m_mutex.lock();
        m_mstate = STATE_BEFORE_TRAJ_FOLLOW;
        m_mutex.unlock();
      }
    }
    else if (m_mstate == STATE_BEFORE_TRAJ_FOLLOW)
    {
      if (m_motionCommand == POSITION_INTF)
      {
        ROS_INFO_STREAM("STATE_BEFORE_TRAJ_FOLLOW: POSITION_INTERFACE");
      }
      else
      {
        ROS_WARN_STREAM("STATE_BEFORE_TRAJ_FOLLOW: VELOCITY_INTERFACE");
      }

      // next state
      m_mutex.lock();
      m_mstate = STATE_TRAJ_FOLLOW;
      m_mutex.unlock();
    }
    else if (m_mstate == STATE_TRAJ_FOLLOW) // exit on finish or idle
    {
      //        ROS_INFO_STREAM("STATE_TRAJ_FOLLOW");
    }
    else if (m_mstate == STATE_FINISH)
    {
      ROS_INFO_STREAM("STATE_FINISH");
    }

    // external set state has precedence over interal new state
    m_mutex.lock();
    if (m_mstatePendFlag)
    {
      m_mstate = m_pend_mstate;
      m_mstatePendFlag = false;
    }
    m_mutex.unlock();
  }

  void ArmCommInterface::executeMState()
  {
    if (m_mstate != STATE_TRAJ_FOLLOW)
    {
      m_ready = false;
    }

    if (m_mstate == STATE_INIT)
    {
    }
    else if (m_mstate == STATE_IDLE)
    {
      qDebug("ArmCommInterface: STATE_IDLE -> stop all script traj threads");
      setMotionCommand(m_motionCommand, false);
      // usleep(2000*1000); // this does not always work, it block the update function too long
      m_mWaitCnt = 0;
    }
    else if (m_mstate == STATE_BEFORE_TRAJ_FOLLOW_WAIT)
    {
      m_mWaitCnt++;
    }
    else if (m_mstate == STATE_BEFORE_TRAJ_FOLLOW)
    {
      m_mutexJointState.lock();
      m_qStart = m_jointState.q();
      m_mutexJointState.unlock();

      m_mutex.lock();
      m_qd = m_qStart;
      m_qpd = VectorDOFArm::Zero();
      m_mutex.unlock();

      m_currentCycle = 0;

      qDebug("ArmCommInterface: STATE_BEFORE_TRAJ_FOLLOW -> start script traj thread...");

      if (!setMotionCommand(m_motionCommand, true))
      {
        // exit with error;
        setFinishMState();
        m_error = true;
        m_errorString = "Setting motion interface failed";
      }
    }
    else if (m_mstate == STATE_TRAJ_FOLLOW)
    {
      m_ready = true;

      if (m_motionCommand == POSITION_INTF)
      {
        m_mutex.lock();
        if (!writeTraj(m_waypointId++, m_qd, 19, 3.1, COMM_PERIOD_2))
          m_error = true;
        m_mutex.unlock();
      }
      else if (m_motionCommand == VELOCITY_INTF)
      {
        //            ROS_INFO_STREAM("m_qpd: "<<m_qpd.transpose());

        m_mutex.lock();
        if (!writeTraj(m_waypointId++, m_qpd, 19, 3.1, COMM_PERIOD_2))
          m_error = true;
        m_qpd.setZero();
        m_mutex.unlock();
      }
      else
      {
        m_error = true;
        m_errorString = "Invalid motion command. Not sending to robot.";
        ROS_ERROR_STREAM(m_errorString.toStdString().c_str());
      }
    }
    else if (m_mstate == STATE_FINISH)
    {
      qDebug("STATE_FINISH");
      setMotionCommand(m_motionCommand, false);
      stop();
    }
  }

  bool ArmCommInterface::parseConfig(const QString &configFilePath)
  {
    // load config file
    yaml::Parameters comm_params("communication");
    if (!comm_params.loadFile(configFilePath.toStdString()))
    {
      ROS_ERROR_STREAM("ArmCommInterface(): Error loading config file: " << configFilePath.toStdString());
      return false;
    }

    // get parameters
    m_pcIp = QString::fromStdString(comm_params.get<std::string>("PC_ADDRESS"));
    m_robotIp = QString::fromStdString(comm_params.get<std::string>("ROBOT_ADDRESS"));
    m_trajPort = comm_params.get<unsigned int>("PC_TRAJ_PORT");
    m_robotStatePort = comm_params.get<unsigned int>("ROBOT_STATE_PORT", 30003);

    QString mComm = QString::fromStdString(comm_params.get<std::string>("MOTION_COMMAND", "VELOCITY"));
    if ((!mComm.compare("VELOCITY")) || (!mComm.compare("velocity")) || (!mComm.compare("vel")) || (!mComm.compare("Velocity")))
    {
      // If velocity interface is not explicitly called then use position interface (safer!)
      m_motionCommand = VELOCITY_INTF;
      ROS_WARN_STREAM("VELOCITY_INTF: " << m_motionCommand);
    }
    else
    {
      m_motionCommand = POSITION_INTF;
    }

    // get the robot version from the robot config; always fall back to version 1.8
    // as this is the version for tomm's arms
    QString robotVersionStr = QString::fromStdString(comm_params.get<std::string>("ROBOT_VERSION", "1.8"));
    if (robotVersionStr == "1.8")
    {
      m_robotVersion = RobotStateClient::VERSION_1V8;
    }
    else if (robotVersionStr == "3.1")
    {
      m_robotVersion = RobotStateClient::VERSION_3V1;
    }
    else if (robotVersionStr == "3.4")
    {
      m_robotVersion = RobotStateClient::VERSION_3V4;
    }
    else if (robotVersionStr == "3.5")
    {
      m_robotVersion = RobotStateClient::VERSION_3V5;
    }
    else
    {
      ROS_ERROR("Unsupported robot version '%s'", robotVersionStr.toLatin1().data());
      ROS_ERROR("Fall back to default robot version 1.8");
      m_robotVersion = RobotStateClient::VERSION_1V8;
      robotVersionStr = "1.8";
    }

    ROS_INFO_STREAM("m_pcIp: " << m_pcIp.toStdString());
    ROS_INFO_STREAM("m_robotIp: " << m_robotIp.toStdString());
    ROS_INFO_STREAM("m_trajPort: " << m_trajPort);
    ROS_INFO_STREAM("m_robotStatePort: " << m_robotStatePort);
    ROS_INFO_STREAM("m_robotVersion: " << robotVersionStr.toStdString());

    return true;
  }

  bool ArmCommInterface::enableTrajScript(MotionCommand cmd, bool enable)
  {
    ur_script_manager::setScriptManagerState srv;

    switch (cmd)
    {
    case POSITION_INTF:
      srv.request.name = "traj_pos";
      break;

    case VELOCITY_INTF:
      srv.request.name = "traj_vel";
      break;

    default:
      srv.request.name = "traj_pos";
      break;
    }

    srv.request.enable = enable;

    if (!m_setScriptState.call(srv))
    {
      ROS_ERROR("ArmCommInterface: Failed to call service 'setScriptManagerState'");
      ROS_ERROR("Node namespace= '%s'", m_ns.toStdString().c_str());
      return false;
    }

    if (!srv.response.ok)
    {
      ROS_ERROR("ArmCommInterface: Calling service 'setScriptManagerState' failed.");
      ROS_ERROR("Node namespace= '%s'", m_ns.toStdString().c_str());
      return false;
    }

    return true;
  }

  bool ArmCommInterface::setMotionCommand(MotionCommand cmd, bool enable)
  {
    if (enable)
    {
      // start the server
      qDebug("Start server ...");
      m_serverTraj = new QTcpServer();
      if (!m_serverTraj->listen(QHostAddress(m_pcIp), m_trajPort))
      {
        ROS_ERROR("ArmCommInterface: Server listen failed with error '%s'",
                  m_serverTraj->errorString().toLatin1().data());
        return false;
      }

      qDebug("Enable script thread traj ...");
      if (!enableTrajScript(cmd, true))
      {
        if (m_serverTraj->isListening())
        {
          m_serverTraj->close();
        }
        delete m_serverTraj;
        m_serverTraj = 0;

        return false;
      }

      qDebug("ArmCommInterface: Waiting for incomming socket client connection ...");
      // the connection should now already be on the server
      if (!m_serverTraj->waitForNewConnection(100))
      {
        ROS_ERROR("ArmCommInterface: Server has no client connection.");

        if (m_serverTraj->isListening())
        {
          m_serverTraj->close();
        }
        delete m_serverTraj;
        m_serverTraj = 0;
        return false;
      }

      qDebug("ArmCommInterface: Get connected client socket.");
      m_socketTraj = m_serverTraj->nextPendingConnection();
      if (m_socketTraj == 0)
      {
        ROS_ERROR("ArmCommInterface: Getting client socket failed");

        if (m_serverTraj->isListening())
        {
          m_serverTraj->close();
        }
        delete m_serverTraj;
        m_serverTraj = 0;
        return false;
      }

      qDebug("ArmCommInterface: Traj socket successfully created.");
    }
    else
    {
      if (!enableTrajScript(cmd, false))
      {
        ROS_ERROR("ArmCommInterface: Stopping script thread traj failed");
      }

      if (m_socketTraj != 0)
      {
        m_socketTraj->close();
        m_socketTraj = 0;
      }

      if (m_serverTraj != 0)
      {
        if (m_serverTraj->isListening())
        {
          m_serverTraj->close();
        }
        delete m_serverTraj;
        m_serverTraj = 0;
      }
    }

    return true;
  }

  bool ArmCommInterface::writeTraj(
      int waypointId,
      const VectorDOFArm &qd,
      double acc,
      double vel,
      double totalTime,
      double round)
  {
    QByteArray data;
    QDataStream s(&data, QIODevice::ReadWrite);

    typedef Eigen::Matrix<int, DOF_ARM, 1> VectorDOFi;
    VectorDOFi qi;

    s << qint32(waypointId);
    for (int i = 0; i < DOF_ARM; i++)
    {
      s << qint32(qd[i] * MULT_jointstate);
      qi(i) = qint32(qd[i] * MULT_jointstate);
    }

    s << qint32(acc * MULT_jointstate) << qint32(vel * MULT_jointstate);
    s << qint32(totalTime * MULT_time) << qint32(round * MULT_blend);

    if (data.length() > 0)
    {
      m_socketTraj->write(data);
      if (!m_socketTraj->waitForBytesWritten())
      {
        qDebug("ArmCommInterface: Write to the socket failed.");
        return false;
      }
    }

    return true;
  }

}
