/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <udp_robot_connection/data/sensor_data.h>
#include <udp_robot_connection/data/utils.h>

#include <cstring>

namespace udp
{

  void SensorData::fsensor(const std::string &name, double data[6])
  {
    for (size_t i = 0; i < fsensors.size(); ++i)
    {
      auto &fs = fsensors[i];
      if (fs.name == name)
      {
        std::memcpy(fs.reading, data, 6 * sizeof(double));
        return;
      }
    }
    ForceSensor fs;
    fs.name = name;
    std::memcpy(fs.reading, data, 6 * sizeof(double));
    fsensors.push_back(fs);
  }

  size_t SensorData::size() const
  {
    return
        // Size of id
        sizeof(uint64_t) +
        // Size of encoders buffer length + data
        sizeof(uint64_t) + encoders.size() * sizeof(double) +
        // Size of encoder velocities buffer length + data
        sizeof(uint64_t) + encoderVelocities.size() * sizeof(double) +
        // Size of torques buffer length + data
        sizeof(uint64_t) + torques.size() * sizeof(double) +
        // Size of fsensors
        fsensorsSize() +
        // Size of orientation (4) + angularVelocity (3) + linearAcceleration (3)
        imu_frame.size() * sizeof(char) + 10 * sizeof(double);
  }

  size_t SensorData::fsensorsSize() const
  {
    size_t ret = sizeof(uint64_t); // Lenght of fSensors
    for (size_t i = 0; i < fsensors.size(); ++i)
    {
      const ForceSensor &fsensor = fsensors[i];
      ret += sizeof(uint64_t) + fsensor.name.size() * sizeof(char) + 6 * sizeof(double);
    }
    return ret;
  }

  namespace utils
  {

    static void toBuffer(uint8_t *dest, const ForceSensor &src, size_t &offset)
    {
      toBuffer(dest, src.name, offset);
      memcpy_advance(dest, src.reading, 6 * sizeof(double), offset);
    }

    static void toBuffer(uint8_t *dest, const std::vector<ForceSensor> &src, size_t &offset)
    {
      uint64_t size = src.size();
      memcpy_advance(dest, &size, sizeof(uint64_t), offset);
      for (size_t i = 0; i < src.size(); i++)
      {
        toBuffer(dest, src[i], offset);
      }
    }

  } // namespace utils

  size_t SensorData::toBuffer(uint8_t *buffer) const
  {
    size_t offset = 0;
    utils::memcpy_advance(buffer, &id, sizeof(uint64_t), offset);
    utils::toBuffer(buffer, encoders, offset);
    utils::toBuffer(buffer, encoderVelocities, offset);
    utils::toBuffer(buffer, torques, offset);
    utils::toBuffer(buffer, fsensors, offset);
    utils::toBuffer(buffer, imu_frame, offset);
    utils::memcpy_advance(buffer, orientation, 4 * sizeof(double), offset);
    utils::memcpy_advance(buffer, angularVelocity, 3 * sizeof(double), offset);
    utils::memcpy_advance(buffer, linearAcceleration, 3 * sizeof(double), offset);
    return offset;
  }

  namespace utils
  {

    static void fromBuffer(ForceSensor &dest, const uint8_t *src, size_t &offset)
    {
      fromBuffer(dest.name, src, offset);
      memcpy_advance(dest.reading, src, 6 * sizeof(double), offset);
    }

    static void fromBuffer(std::vector<ForceSensor> &dest, const uint8_t *src, size_t &offset)
    {
      uint64_t size = 0;
      memcpy_advance(&size, src, sizeof(uint64_t), offset);
      dest.resize(size);
      for (size_t i = 0; i < dest.size(); i++)
      {
        fromBuffer(dest[i], src, offset);
      }
    }

  } // namespace utils

  size_t SensorData::fromBuffer(uint8_t *buffer)
  {
    size_t offset = 0;
    utils::memcpy_advance(&id, buffer, sizeof(uint64_t), offset);
    utils::fromBuffer(encoders, buffer, offset);
    utils::fromBuffer(encoderVelocities, buffer, offset);
    utils::fromBuffer(torques, buffer, offset);
    utils::fromBuffer(fsensors, buffer, offset);
    utils::fromBuffer(imu_frame, buffer, offset);
    utils::memcpy_advance(orientation, buffer, 4 * sizeof(double), offset);
    utils::memcpy_advance(angularVelocity, buffer, 3 * sizeof(double), offset);
    utils::memcpy_advance(linearAcceleration, buffer, 3 * sizeof(double), offset);
    return offset;
  }

} // namespace mc_udp
