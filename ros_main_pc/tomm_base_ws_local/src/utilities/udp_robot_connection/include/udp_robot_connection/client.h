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

  /** 
   * @brief Client class
   * 
   * Implement a simple UDP server of sending robot sensors' data and receiving
   * control data
   *
   * Filling the sensors' data and making sure of the time-consistency of
   * sensors' and control data is left to clients of this class
   *
   */
  struct Client
  {
    /**
     * @brief Construct a new Client object
     * 
     * @param host        host address
     * @param port        udp port
     * @param timeout_us  timeout in us for blocking calls
     */
    Client(const std::string & host, int port, long int timeout_us=-1);

    /**
     * @brief Destroy the Client object
     * 
     */
    ~Client();

    /**
     * @brief send hello bytes to test connection
     * 
     */
    void sendHello();

    /** 
     * @brief Should be send when we are ready to receive more data after the first sensors 
     */
    void init();

    /**
     * @brief Receive sensor data (non blocking function)
     * 
     * @return true new data
     * @return false no new data
     */
    bool recv();

    bool recvBlocking();

    /** 
     * @brief Send control data
     *
     * Does not modify the control id
     */
    void send();

    CommandData & command(){return command_;}

    const SensorData & sensor() const {return sensor_;}

  private:
    CommandData command_;
    SensorData sensor_;

    int socket_;
    sockaddr_in server_;
    std::vector<uint8_t> recvData_;
    std::vector<uint8_t> sendData_;
    sockaddr_in recvAddr_;
    socklen_t recvAddrLen_;
  };

} // namespace mc_udp
