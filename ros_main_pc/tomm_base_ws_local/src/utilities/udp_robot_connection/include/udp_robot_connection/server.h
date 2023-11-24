/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <netinet/in.h>
#include <sys/socket.h>

#include <udp_robot_connection/data/command_data.h>
#include <udp_robot_connection/data/sensor_data.h>

namespace udp
{

  /** Implement a simple UDP server of sending robot sensors' data and receiving
   * control data
   *
   *  Filling the sensors' data and making sure of the time-consistency of
   *  sensors' and control data is left to clients of this class
   *
   */
  struct Server
  {
    enum State {DISCONNECTED, WAITING, CONNECTED};

    /**
     * @brief Construct a new Server object
     * 
     */
    Server();

    /**
     * @brief Construct a new Server object
     * 
     * @param port        udp port
     * @param timeout_us  timeout in us for blocking calls
     */
    Server(int port, long int timeout_us=-1);

    /**
     * @brief Destroy the Server object
     * 
     */
    ~Server();

    /** 
     * @brief Receive command data (non blocking function)
     * 
     * @return true new data
     * @return false no data
     */
    bool recv();

    bool recvBlocking();

    /**
     * @brief send the sensor data
     * 
     */
    void send();

    /**
     * @brief read the command data
     * 
     * @return const CommandData& 
     */
    const CommandData &command() const {return command_;}

    /**
     * @brief access the sensor data
     * 
     * @return SensorData& 
     */
    SensorData &sensor() {return sensor_;}

    /**
     * @brief shutdown connection 
     */
    void stop();

    /**
     * @brief restart conneciton
     * 
     * @param port 
     * @param timeout_us
     */
    void restart(int port, long int timeout_us=-1);

  private:
    void start(int port, long int timeout_us);

  private:
    State state_;
    CommandData command_;
    SensorData sensor_;

    int socket_;

    sockaddr_in client_;
    socklen_t clientAddrLen_;

    std::vector<uint8_t> recvData_;
    std::vector<uint8_t> sendData_;

    std::string id_;
  };

}
