#ifndef TOMM_HARDWARE_DATAUNPACKER_H
#define TOMM_HARDWARE_DATAUNPACKER_H

#include <QString>
#include <tomm_hardware_real/utilities/joint_state.h>

namespace tomm_hw
{

  class DataUnpacker
  {

  public:
    enum MsgHeader
    {
      MSG_OUT = 1,               //robot side: string message
      MSG_QUIT = 2,              //robot side: quit flag
      MSG_JOINT_STATES = 3,      //robot side: info of joint states
      MSG_MOVEJ = 4,             //computer side: command of movej
      MSG_WAYPOINT_FINISHED = 5, //robot side
      MSG_STOPJ = 6,             //computer side: stop
      MSG_SERVOJ = 7,            //TODO to check the ros package driver.py
      MSG_PARK_ROBOT = 8,
      MSG_GO_HOME = 9,
      MSG_ENTER_TRAJ_FOLLOW = 10,
      MSG_EXIT_TRAJ_FOLLOW = 11
    };

    static const double COMM_PERIOD;
    static const double COMM_PERIOD_2;
    static const double MULT_jointstate;
    static const double MULT_time;
    static const double MULT_blend;

  private:
    QByteArray pendingData;
    int wayPointId;

    double q[DOF_ARM];
    double qdot[DOF_ARM];
    double tau[DOF_ARM];
    int q_raw[DOF_ARM];
    int qdot_raw[DOF_ARM];
    int tau_raw[DOF_ARM];
    JointState<DOF_ARM> q_out;

    QString msg;
    bool m_ready;

  public:
    DataUnpacker();
    void appendNewData(QByteArray &newData);
    void unpackData();
    void getJointStates(double q[DOF_ARM], double qdot[DOF_ARM], double tau[DOF_ARM]);
    const JointState<DOF_ARM>& getJointState();
    bool hasJointStatesInfo();
    QString getMsg();
    int getWayPointId();
    bool isReady() const;
    
  };
}

#endif // DATAUNPACKER_H
