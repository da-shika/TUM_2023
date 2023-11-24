/*! \file
 *
 * \author Simon Armleder
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#ifndef CONTROL_CORE_BODY_ID_H
#define CONTROL_CORE_BODY_ID_H

#include <string>
#include <vector>

namespace control_core
{

  /*!
  * \brief The BodyId definition.
  *
  * Identifier for the Foot.
  *
  */
  class BodyId
  {
  public:
    /**
     * @brief body id as enum
     */
    enum Value
    {
      LEFT_FOOT = 0,
      RIGHT_FOOT,
      LEFT_HAND,
      RIGHT_HAND,
      LEFT_ELBOW,
      RIGHT_ELBOW,
      TORSO,
      HEAD,
      UNKNOWN
    };

    /**
     * @brief bodies as std::string
     */
    const static std::vector<std::string>& Names();
    const static std::vector<std::string>& FootNames();
    const static std::vector<std::string>& HandNames();
    const static std::vector<std::string>& ElbowNames();
    const static std::vector<std::string>& EndEffectorsNames();
    const static std::vector<std::string>& UpperBodyNames();

    /**
     * @brief bodies as Value
     */
    const static std::vector<Value>& Ids();
    const static std::vector<Value>& FootIds();
    const static std::vector<Value>& HandIds();
    const static std::vector<Value>& ElbowIds();
    const static std::vector<Value>& EndEffectorsIds();
    const static std::vector<Value>& UpperBodyIds();

  public:
    BodyId() = default;

    BodyId(Value v) : v_(v)
    {
    }

    BodyId(int v) : v_(static_cast<Value>(v))
    {
    }

    BodyId other() const
    {
      if (v_ == LEFT_FOOT)
        return BodyId(RIGHT_FOOT);
      if (v_ == RIGHT_FOOT)
        return BodyId(LEFT_FOOT);
      if (v_ == LEFT_HAND)
        return BodyId(RIGHT_HAND);
      if (v_ == RIGHT_HAND)
        return BodyId(LEFT_HAND);
      if (v_ == LEFT_ELBOW)
        return BodyId(LEFT_ELBOW);
      if (v_ == RIGHT_ELBOW)
        return BodyId(RIGHT_ELBOW);
      return BodyId(UNKNOWN);
    }

    bool isLeftFoot() const
    {
      return v_ == LEFT_FOOT;
    }

    bool isRightFoot() const
    {
      return v_ == RIGHT_FOOT;
    }

    bool isLeftHand() const
    {
      return v_ == LEFT_HAND;
    }

    bool isRightHand() const
    {
      return v_ == RIGHT_HAND;
    }

    std::string toString() const
    {
      if (v_ == LEFT_FOOT)
        return "LEFT_FOOT";
      if (v_ == RIGHT_FOOT)
        return "RIGHT_FOOT";
      if (v_ == LEFT_HAND)
        return "LEFT_HAND";
      if (v_ == RIGHT_HAND)
        return "RIGHT_HAND";
      if (v_ == LEFT_ELBOW)
        return "LEFT_ELBOW";
      if (v_ == RIGHT_ELBOW)
        return "RIGHT_ELBOW";
      if (v_ == TORSO)
        return "TORSO";
      if (v_ == HEAD)
        return "HEAD";
      return "UNKNOWN";
    }
    
    operator BodyId::Value() const
    {
      return v_;
    }

  private:
    Value v_;
  };

}

#endif // CONTROL_CORE_BODY_ID_H
