#include <udp_robot_connection/server.h>

#include <chrono>

#include <ros/ros.h>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "test_server_node");

  int port = 4444;
  if(argc > 1)
  {
    port = std::atoi(argv[1]);
  }

  udp::Server server(port);

  auto& sensor = server.sensor();
  sensor.encoders = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  sensor.torques = {100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110};
  double reading[6];
  for(size_t i = 0; i < 6; ++i)
    reading[i] = i + 1;
  sensor.fsensor("rfsensor", reading);
  sensor.fsensor("lfsensor", reading);
  sensor.fsensor("rhsensor", reading);
  sensor.fsensor("lhsensor", reading);
  sensor.imu_frame = "base_link";
  sensor.orientation[0] = 1;
  sensor.orientation[1] = 2;
  sensor.orientation[2] = 3;
  sensor.orientation[3] = 4;
  sensor.angularVelocity[0] = 5;
  sensor.angularVelocity[1] = 6;
  sensor.angularVelocity[2] = 7;
  sensor.linearAcceleration[0] = 8;
  sensor.linearAcceleration[1] = 9;
  sensor.linearAcceleration[2] = 10;
  sensor.id = 0;

  size_t command_id = 0;
  while(1)
  {
    // stream data as fast as possible
    auto start_t = std::chrono::system_clock::now();
    server.send();
    auto send_t = std::chrono::system_clock::now();

    // keep pulling for new data
    if(server.recv())
    {
      const auto& command = server.command();
      command_id = command.id;
      // if(command.id != sensor.id - 1)
      // {
      //   ROS_WARN_STREAM("Server command id " << command.id << " does not match sensors id " << sensor.id);
      // }
    
      auto recv_t = std::chrono::system_clock::now();
      auto send_us = double(std::chrono::duration_cast<std::chrono::nanoseconds>(send_t - start_t).count())/1e3;
      auto recv_us = double(std::chrono::duration_cast<std::chrono::nanoseconds>(recv_t - send_t).count())/1e3;

      if(send_us > 1e3 || recv_us > 1e3)
      {
        ROS_ERROR_STREAM(
          "send_dt: " << send_us << " us, " << 
          "recv_dt: " << recv_us << " us, control_id: " << command_id);
      }
      else 
      {
        ROS_INFO_STREAM(
          "send_dt: " << send_us << " us, " << 
          "recv_dt: " << recv_us << " us, control_id: " << command_id);
      }
      sensor.id += 1;
    }
    
    // 2k rate
    usleep(500);
  }

  return 0;
};