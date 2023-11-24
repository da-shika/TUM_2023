#ifndef TOMM_HARDWARE_TCPSOCKETTOOLS_H
#define TOMM_HARDWARE_TCPSOCKETTOOLS_H

#include <QTcpSocket>
#include <QHostAddress>
#include <ros/ros.h>

#include <tomm_hardware_real/utilities/joint_state.h>

namespace tomm_hw
{

  class TcpSocketTools
  {
  public:
    enum Identity
    {
      BAD_IDENTITY,
      SERVER,
      CLIENT
    };

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
      MSG_EXIT_TRAJ_FOLLOW = 11,
      MSG_ENTER_TRAJ_FOLLOW_V = 12,
      MSG_EXIT_TRAJ_FOLLOW_V = 13
    };

  private:
    QTcpSocket *m_socket;

  public:
    TcpSocketTools(QTcpSocket *socket);
    ~TcpSocketTools();

    QString readString();
    QByteArray readByteArrayAll();
    QByteArray readByteArray(int size);
    void writeString(const QString &line);
    void writeByteArray(const QByteArray &data);
    void writeCommand(MsgHeader msg);
    void writeTraj(int waypointId,
                   const VectorDOFArm &qd,
                   double acc = 3.0,
                   double vel = 0.75,
                   double totalTime = 0.0,
                   double round = 0.0);

    QByteArray genMsgPacket(MsgHeader msg);
  };

}

#endif // TCPSOCKETTOOLS_H
