#include <tomm_hardware_real/utilities/tcp_socket_tools.h>
#include <tomm_hardware_real/utilities/data_unpacker.h>

#include <QTextStream>
#include <QFile>
#include <QDataStream>

#define DEBUG_DATA

namespace tomm_hw
{

  TcpSocketTools::TcpSocketTools(QTcpSocket *socket) : m_socket(socket)
  {
  }

  TcpSocketTools::~TcpSocketTools()
  {
  }

  QByteArray TcpSocketTools::readByteArrayAll()
  {
    m_socket->waitForReadyRead(0);
    int bytesAvail = m_socket->bytesAvailable();
    if (bytesAvail > 0)
    {
      return m_socket->readAll();
    }
    else
    {
      QByteArray a;
      return a;
    }
  }

  void TcpSocketTools::writeString(const QString &line)
  {
    if (line.length() > 0)
    {
      m_socket->write(line.toLatin1());
      if (!m_socket->waitForBytesWritten())
      {

        qDebug() << "Write to the socket failed";
      }
    }
  }

  void TcpSocketTools::writeByteArray(const QByteArray &data)
  {
    if (data.length() > 0)
    {
      m_socket->write(data);
      if (!m_socket->waitForBytesWritten())
      {

        qDebug() << "Server - write to the socket failed";
      }
    }
  }

  void TcpSocketTools::writeCommand(MsgHeader msg)
  {
    QByteArray cmdData;
    QDataStream s(&cmdData, QIODevice::ReadWrite);
    s << qint32(msg);
    writeByteArray(cmdData);
  }

  void TcpSocketTools::writeTraj(int waypointId, const VectorDOFArm &qd, double acc, double vel, double totalTime, double round)
  {
    QByteArray cmdData;
    QDataStream s(&cmdData, QIODevice::ReadWrite);

    s << qint32(waypointId);
    for (int i = 0; i < DOF_ARM; i++)
    {
      s << qint32(qd[i] * DataUnpacker::MULT_jointstate);
    }

    s << qint32(acc * DataUnpacker::MULT_jointstate) << qint32(vel * DataUnpacker::MULT_jointstate);
    s << qint32(totalTime * DataUnpacker::MULT_time) << qint32(round * DataUnpacker::MULT_blend);

    writeByteArray(cmdData);
  }

  QByteArray TcpSocketTools::genMsgPacket(MsgHeader msg)
  {
    QByteArray cmdData;
    QDataStream s(&cmdData, QIODevice::ReadWrite);
    s << qint32(msg);
    return cmdData;
  }

}