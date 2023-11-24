/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <udp_robot_connection/client.h>

#include <ros/ros.h>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "test_client_node");

  std::string host = "localhost"; // "10.68.0.1"; // localhost
  int port = 4444;
  if(argc > 1)
  {
    host = argv[1];
  }
  if(argc > 2)
  {
    port = std::atoi(argv[2]);
  }
  udp::Client client(host, port);

  auto & command = client.command();
  command.encoders.resize(30, 0);
  uint64_t prev_id = 0;

  bool init = false;
  while(1)
  {
    // sleep till next command
    if(client.recv())
    {
      if(!init)
      {
        init = true;
        client.init();
      }
      
      const auto & sensor = client.sensor();
      if(sensor.id != prev_id + 1)
      {
        ROS_WARN_STREAM("Missed one frame of sensors data (id: " << sensor.id << ", previous id: " << prev_id << ")");
      }
      command.id = sensor.id;
      prev_id = command.id;

      // std::cout << client.sensor().imu_frame << std::endl;
      // for(const auto & fs : client.sensor().fsensors)
      // {
      //  std::cout << "- " << fs.name << ": " << fs.reading[0] << ", "
      //                                       << fs.reading[1] << ", "
      //                                       << fs.reading[2] << ", "
      //                                       << fs.reading[3] << ", "
      //                                       << fs.reading[4] << ", "
      //                                       << fs.reading[5] << "\n";
      // }
      ROS_INFO_STREAM("sensors.id: " << sensor.id);
      client.send();
    }
  }
  
  return 0;
}
