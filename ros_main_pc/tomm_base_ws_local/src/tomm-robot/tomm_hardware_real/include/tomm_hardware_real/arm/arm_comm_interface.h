#ifndef TOMM_HARDWARE_ARM_INTERFACE_H
#define TOMM_HARDWARE_ARM_INTERFACE_H

#include <QThread>
#include <QTcpServer>
#include <QMutex>

#include <tomm_hardware_real/utilities/tcp_socket_tools.h>
#include <tomm_hardware_real/utilities/robot_state_client.h>

#include <ur_script_manager/setScriptManagerState.h>
#include <ur_script_manager/getScriptManagerStates.h>

#include <RtThreads/Thread.h>

namespace tomm_hw
{
  typedef RtThreads::Thread TThread;

  class ArmCommInterface : TThread
  {
  public:
    enum State
    {
      STATE_INIT = 0,
      STATE_IDLE,
      STATE_BEFORE_TRAJ_FOLLOW_WAIT,
      STATE_BEFORE_TRAJ_FOLLOW,
      STATE_TRAJ_FOLLOW,
      STATE_FINISH
    };

    enum MotionCommand
    {
      POSITION_INTF = 1,
      VELOCITY_INTF
    };

    static const double COMM_PERIOD;
    static const double COMM_PERIOD_2;
    static const double MULT_jointstate;
    static const double MULT_time;
    static const double MULT_blend;

    // Member variables
  private:
    QString m_ns;
    ros::NodeHandle m_node;
    ros::ServiceClient m_setScriptState;

    QTcpServer *m_serverTraj;
    QTcpSocket *m_socketTraj;

    // IP Addresses
    QString m_pcIp;
    QString m_robotIp;

    // Ports
    quint16 m_trajPort;       //   e.g. 50002 To send joint desired state (these two ports are defined inside the script file)
    quint16 m_robotStatePort; // 30003 get robot state from real time module on robot

    // robot version for robot state client
    RobotStateClient::Version m_robotVersion;

    JointState<DOF_ARM> m_jointState;
    VectorDOFArm m_qStart;
    VectorDOFArm m_qd;  // Desired joint position for the UR controller
    VectorDOFArm m_qpd; // Desired joint velocity for the UR controller
    int m_currentCycle;
    int m_waypointId;

    // Error handlers
    QString m_errorString;
    bool m_error;
    bool m_stopThread;
    bool m_stoppedThread;
    bool m_ready;

    int m_cnt;

    // State MachineState
    State m_mstate;
    State m_pend_mstate;
    bool m_mstatePendFlag;

    int m_mWaitCnt;

    QMutex m_mutex;

    QMutex m_mutexJointState;

    RobotStateClient *m_robotStateClient;
    bool m_extRobotSateClientFlag;

    MotionCommand m_motionCommand;

    // Member functions
  public:
    ArmCommInterface(const QString &ns,
                  const QString &pcIpAddr,
                  const quint16 trajPort,
                  const QString &robotIpAddr,
                  const quint16 robotStatePort = 30003,
                  QObject *parent = 0);

    // Added option to link an external robotStateClient
    ArmCommInterface(const QString &ns,
                  const QString &configFilePath,
                  RobotStateClient *extRobotStateClient = NULL);

    ~ArmCommInterface();

    bool error();
    const QString &errorString();

    JointState<DOF_ARM> getCurrentJointState();
    void setQd(const VectorDOFArm &qTarget, const VectorDOFArm &qpTarget = VectorDOFArm::Zero());

    void setIdleMState();
    void setFinishMState();
    State getMState();

    bool robotStateIsReady() const;
    bool isReady() const;

    const MotionCommand getMotionIntf() const;
    void setMotionIntf(const MotionCommand intf);

  private:
    bool init();
    void run();
    void stop(); // Stop thread and close sockets and ports (on PC side only)
    void cleanup();

    void updateStateMachine();
    void executeMState();

    bool parseConfig(const QString &configFilePath);

    // start and stop the thread in the robot script
    bool enableTrajScript(MotionCommand cmd, bool enable);

    // starts / stops the motion interface and the server
    // never start both and never start before stop
    bool setMotionCommand(MotionCommand cmd, bool enable);

    // send pos / vel to robot
    bool writeTraj(
        int waypointId,
        const VectorDOFArm &qd,
        double acc = 3.0,
        double vel = 0.75,
        double totalTime = 0.0,
        double round = 0.0);
  };

}

#endif // ARM_INTERFACE_H
