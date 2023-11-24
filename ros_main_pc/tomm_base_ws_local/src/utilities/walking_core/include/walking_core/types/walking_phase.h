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

#ifndef WALKING_CORE_WALKING_PHASE_H
#define WALKING_CORE_WALKING_PHASE_H

#include <string>

#include <walking_core_msgs/WalkingPhase.h>

namespace control_core
{

  /*!
  * \brief The WalkingPhase definition.
  *
  * Identifier for the WalkingPhase.
  *
  */
  class WalkingPhase
  {
  public:
    enum Value
    {
      STANCE = 0,
      DOUBLESUPPORT = 1,
      SINGLESUPPORT = 2
    };

  public:
    WalkingPhase() = default;

    WalkingPhase(Value v) : v_(v)
    {
    }

    WalkingPhase other() const
    {
      if (v_ == DOUBLESUPPORT)
        return WalkingPhase(SINGLESUPPORT);
      else if(v_ == SINGLESUPPORT)
        return WalkingPhase(DOUBLESUPPORT);
      else
        return WalkingPhase(STANCE);
    }

    WalkingPhase& operator=(const walking_core_msgs::WalkingPhase& msg)
    {
      v_ = static_cast<Value>(msg.phase);
      return *this;
    }

    operator walking_core_msgs::WalkingPhase() const
    {
      walking_core_msgs::WalkingPhase msg;
      msg.phase = v_;
      return msg;
    } 

    bool isStance() const
    {
      return v_ == STANCE;
    }

    bool isDoubleSupport() const
    {
      return v_ == DOUBLESUPPORT;
    }

    bool isSingleSupport() const
    {
      return v_ == SINGLESUPPORT;
    }

    std::string toString() const
    {
      if (v_ == DOUBLESUPPORT)
        return "singlesupport";
      else if(v_ == SINGLESUPPORT)
        return "doublesupport";
      else
        return "stance";
    }
    
    operator WalkingPhase::Value() const
    {
      return v_;
    }

  private:
    Value v_;
  };

}

#endif // CONTROL_CORE_FOOT_ID_H
