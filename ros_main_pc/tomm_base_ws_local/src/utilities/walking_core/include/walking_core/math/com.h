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

#ifndef WALKING_CORE_COM_H_
#define WALKING_CORE_COM_H_

#include <control_core/types.h>

namespace cc 
{

  /**
   * @brief convert com state into cop/zmp
   * 
   * computes the cop based on LIPM equation
   * p = x - omega2 * \ddot{x}
   * 
   * @param com_ref 
   * @return cc::Vector3 
   */
  inline cc::LinearPosition com_to_cop(const cc::LinearPosition& com, const cc::LinearAcceleration& com_acc)
  {
    cc::Scalar omega2 = CC_GRAVITY/com.z();
    cc::LinearPosition cop = com - com_acc/omega2;
    cop.z() = 0;
    return cop;
  }

  /**
   * @brief convert com state to dcm
   * 
   * @param com com position
   * @param com_vel com velocity
   * @param z0 ground height
   * @return cc::LinearPosition 
   */
  inline cc::LinearPosition com_to_dcm(const cc::LinearPosition& com, const cc::LinearVelocity& com_vel, cc::Scalar z0 = 0.0)
  {
    cc::Scalar omega = std::sqrt(CC_GRAVITY/(com.z() - z0));

    cc::LinearPosition dcm = com + com_vel/omega;
    dcm.z() = com.z();
    return dcm;
  }

  inline cc::LinearPosition com_to_dcm(const cc::LinearState& com, cc::Scalar z0 = 0.0)
  {
    // compute the dcm from com state
    // dcm = com + omega*\dot{x}
    return com_to_dcm(com.pos(), com.vel(), z0);
  }

}

#endif