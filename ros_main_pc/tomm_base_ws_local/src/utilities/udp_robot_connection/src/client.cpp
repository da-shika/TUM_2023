/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <udp_robot_connection/client.h>

#include <udp_robot_connection/data/hello_data.h>
#include <udp_robot_connection/data/init_data.h>

#include <ros/console.h>

#include <errno.h>
#include <stdexcept>
#include <string.h>
#include <netdb.h>

namespace udp
{

  Client::Client(const std::string &host, int port, long int timeout_us) : 
    recvData_(2048, 0), 
    sendData_(2048, 0)
  {
    // open socket
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0)
    {
      ROS_ERROR_STREAM("Failed to create socket: " << strerror(errno));
      throw std::runtime_error(std::string(strerror(errno)));
    }

    // stepup priority
    int priority = 6;
    if(setsockopt(socket_, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority)) < 0)
    {
      ROS_ERROR_STREAM("Failed to set socket options: " << strerror(errno));
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

    // bind to host
    auto hp = gethostbyname(host.c_str());
    if (!hp)
    {
      ROS_ERROR_STREAM("Failed to resolve host (" << host << "): " << strerror(h_errno));
      throw std::runtime_error(std::string(strerror(errno)));
    }
    memset(&server_, 0, sizeof(server_));
    server_.sin_family = AF_INET;
    server_.sin_port = htons(port);
    memcpy(&server_.sin_addr, hp->h_addr_list[0], hp->h_length);

    // do inital handshake
    sendHello();
  }

  Client::~Client()
  {
    if (socket_ != 0)
    {
      close(socket_);
    }
  }

  bool Client::recv()
  {
    // read the data into receive buffer
    int sz = recvfrom(socket_, recvData_.data(), recvData_.size(), MSG_DONTWAIT, NULL, NULL);
    if (sz >= static_cast<int>(recvData_.size()))
    {
      ROS_WARN_STREAM("Receive buffer was too small to get it all");
      recvData_.resize(recvData_.size() * 2);
    }
    else if (sz > 0)
    {
      sensor_.fromBuffer(recvData_.data());

      // in case there is more: keep reading until we have the newest frame
      do
      {
        sz = recvfrom(socket_, recvData_.data(), recvData_.size(), MSG_DONTWAIT, NULL, NULL);
        if (sz > 0)
        {
          sensor_.fromBuffer(recvData_.data());
          ROS_INFO_STREAM("More data in buffer sensor_id=" << sensor_.id);    // TODO
        }
      } while (sz > 0);

      return true;
    }
    return false;
  }

  bool Client::recvBlocking()
  {
    // read the data into receive buffer
    int sz = recvfrom(socket_, recvData_.data(), recvData_.size(), 0, NULL, NULL);
    if (sz >= static_cast<int>(recvData_.size()))
    {
      ROS_WARN_STREAM("Receive buffer was too small to get it all");
      recvData_.resize(recvData_.size() * 2);
    }
    else if (sz > 0)
    {
      sensor_.fromBuffer(recvData_.data());
      return true;
    }
    return false;
  }

  void Client::sendHello()
  {
    sendto(socket_, &HelloData, sizeof(HelloData), 0, (struct sockaddr *)&server_, sizeof(server_));
  }

  void Client::init()
  {
    sendto(socket_, &InitData, sizeof(InitData), 0, (struct sockaddr *)&server_, sizeof(server_));
  }

  void Client::send()
  {
    auto sz = command_.size();
    if (sz > sendData_.size())
    {
      sendData_.resize(2 * sendData_.size());
      ROS_INFO_STREAM("Needed to resize sending buffer");
    }
    command_.toBuffer(sendData_.data());
    sendto(socket_, sendData_.data(), sz, 0, (struct sockaddr *)&server_, sizeof(server_));
  }

} // namespace mc_udp
