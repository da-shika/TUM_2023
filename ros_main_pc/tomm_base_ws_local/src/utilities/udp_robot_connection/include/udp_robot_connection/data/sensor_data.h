/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <stdint.h>
#include <string>
#include <vector>

namespace udp
{

  /** Represent a force sensor reading */
  struct ForceSensor
  {
    std::string name;
    double reading[6];
  };

  /**
   * @brief Robot State Data
   * 
   */
  struct SensorData
  {
    /** Unique identifier for the reading */
    uint64_t id;

    /** Joint encoders provided in the robot reference order */
    std::vector<double> encoders;

    /** Joint encoders velocity in the robot reference order */
    std::vector<double> encoderVelocities;

    /** Joint torques provided in the robot reference order */
    std::vector<double> torques;

    /** Force sensors readings */
    std::vector<ForceSensor> fsensors;

    /** imu frame */
    std::string imu_frame;

    /** Orientation sensor reading */
    double orientation[4];

    /** Angular velocity sensor */
    double angularVelocity[3];

    /** Linear acceleration */
    double linearAcceleration[3];

    /** Add a force sensor reading */
    void fsensor(const std::string & name, double data[6]);

    /** Compute required buffer size */
    size_t size() const;
    /** Fill pre-allocated buffer, must be of (at-least) size()
     *
     * Returns the number of bytes written */
    size_t toBuffer(uint8_t * buffer) const;
    /** Fill from a provided buffer
     *
     * Returns the number of bytes consumed */
    size_t fromBuffer(uint8_t * buffer);

  private:
    /** Size of required buffer size for force sensors */
    size_t fsensorsSize() const;
  };

} // namespace mc_udp
