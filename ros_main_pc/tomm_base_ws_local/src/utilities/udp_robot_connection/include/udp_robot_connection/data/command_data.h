/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <stdint.h>
#include <string>
#include <vector>

namespace udp
{

  /**
   * @brief CommandData
   * 
   */
  struct CommandData
  {
    /** Unique identifier for the control, should repeat the same as the sensors' reading */
    uint64_t id;

    /** Joint encoders provided in the robot reference order */
    std::vector<double> encoders;

    /** Desired encoder velocity provided in the robot reference order */
    std::vector<double> encoderVelocities;

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
  };

  

} // namespace mc_udp
