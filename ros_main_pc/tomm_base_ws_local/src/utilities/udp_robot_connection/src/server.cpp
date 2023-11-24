/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <udp_robot_connection/server.h>

#include <udp_robot_connection/data/hello_data.h>
#include <udp_robot_connection/data/init_data.h>

#include <ros/console.h>

#include <errno.h>
#include <stdexcept>
#include <string.h>
#include <unistd.h>
#include <chrono>

namespace udp
{

  Server::Server() : 
    socket_(0), 
    state_(DISCONNECTED), 
    recvData_(1024, 0), 
    sendData_(1024, 0)
  {
  }

  Server::Server(int port, long int timeout_us) : 
  socket_(0), 
  state_(DISCONNECTED), 
  recvData_(1024, 0), 
  sendData_(1024, 0)
  {
    start(port, timeout_us);
  }

  Server::~Server()
  {
    stop();
  }

  bool Server::recv()
  {
    int length
      = recvfrom(socket_, recvData_.data(), recvData_.size(), MSG_DONTWAIT, (struct sockaddr *)&client_, &clientAddrLen_);

    if(length > 0)
    {
      if(length == sizeof(HelloData) * sizeof(uint8_t))
      {
        ROS_INFO_STREAM(id_ << " New client sending data");
        state_ = WAITING;
      }
      else if(length == sizeof(InitData) * sizeof(uint8_t))
      {
        ROS_INFO_STREAM(id_ << " Start streaming data to client");
        sensor_.id = 0;
        state_ = CONNECTED;
      }
      else if(length >= static_cast<int>(recvData_.size()))
      {
        ROS_WARN_STREAM(id_ << " Received exactly the buffer size, resizing for next round");
        recvData_.resize(2 * recvData_.size());
      }
      else
      {
        command_.fromBuffer(recvData_.data());
        return true;
      }
    }
    return false;
  }

  bool Server::recvBlocking()
  {
    int length
      = recvfrom(socket_, recvData_.data(), recvData_.size(), 0, (struct sockaddr *)&client_, &clientAddrLen_);

    if(length > 0)
    {
      if(length == sizeof(HelloData) * sizeof(uint8_t))
      {
        ROS_INFO_STREAM(id_ << " New client sending data");
        state_ = WAITING;
      }
      else if(length == sizeof(InitData) * sizeof(uint8_t))
      {
        ROS_INFO_STREAM(id_ << " Start streaming data to client");
        sensor_.id = 0;
        state_ = CONNECTED;
      }
      else if(length >= static_cast<int>(recvData_.size()))
      {
        ROS_WARN_STREAM(id_ << " Received exactly the buffer size, resizing for next round");
        recvData_.resize(2 * recvData_.size());
      }
      else
      {
        command_.fromBuffer(recvData_.data());
        return true;
      }
    }
    return false;
  }

  void Server::send()
  {
    size_t sz = sensor_.size();
    if(sz > sendData_.size())
    {
      ROS_WARN_STREAM(id_ << " Send data buffer is too small for required sending (size: " << sendData_.size()
                        << ", required: " << sz << ")");
      sendData_.resize(sz);
    }
    sensor_.toBuffer(sendData_.data());
    
    if(state_ == WAITING || state_ == CONNECTED)
    {
      sendto(socket_, sendData_.data(), sz, 0, (struct sockaddr *)&client_, clientAddrLen_);
    }
  }

  void Server::stop()
  {
    if(socket_ != 0)
    {
      close(socket_);
    }
    state_ = DISCONNECTED;
  }

  void Server::restart(int port, long int timeout_us)
  {
    stop();
    start(port, timeout_us);
  }

  void Server::start(int port, long int timeout_us)
  {
    // open the socket
    std::stringstream ss;
    ss << "[UDP::" << port << "]";
    id_ = ss.str();
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if(socket_ < 0)
    {
      ROS_ERROR_STREAM("Failed to create socket: " << strerror(errno));
      throw std::runtime_error(std::string(strerror(errno)));
    }

    // stepup priority
    int priority = 6;
    if(setsockopt(socket_, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority)) < 0)
    {
      ROS_ERROR_STREAM("Failed to set socket 'SO_PRIORITY' options: " << strerror(errno));
      throw std::runtime_error(std::string(strerror(errno)));
    }

    // setup timout for blocking recvfrom() function call
    if(timeout_us > 0)
    {
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = timeout_us;
      if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        ROS_ERROR_STREAM("Failed to set socket 'SO_RCVTIMEO' options: " << strerror(errno));
        throw std::runtime_error(std::string(strerror(errno)));
      }
    }

    // bind socket
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    int err = bind(socket_, (struct sockaddr *)&addr, sizeof(addr));
    if(err < 0)
    { 
      ROS_ERROR_STREAM("Failed bind the socket: " << strerror(errno));
      throw std::runtime_error(std::string(strerror(errno)));
    }
    clientAddrLen_ = sizeof(client_);
    state_ = DISCONNECTED;
  }

} // namespace mc_udp
