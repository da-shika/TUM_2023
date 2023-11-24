/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <udp_robot_connection/data/command_data.h>
#include <udp_robot_connection/data/utils.h>

#include <cstring>

namespace udp
{

  size_t CommandData::size() const
  {
    return sizeof(uint64_t) + sizeof(uint64_t) + encoders.size() * sizeof(double) + sizeof(uint64_t) + encoderVelocities.size() * sizeof(double);
  }

  size_t CommandData::toBuffer(uint8_t *buffer) const
  {
    size_t offset = 0;
    utils::memcpy_advance(buffer, &id, sizeof(uint64_t), offset);
    utils::toBuffer(buffer, encoders, offset);
    utils::toBuffer(buffer, encoderVelocities, offset);
    return offset;
  }

  size_t CommandData::fromBuffer(uint8_t *buffer)
  {
    size_t offset = 0;
    utils::memcpy_advance(&id, buffer, sizeof(uint64_t), offset);
    utils::fromBuffer(encoders, buffer, offset);
    utils::fromBuffer(encoderVelocities, buffer, offset);
    return offset;
  }

}
