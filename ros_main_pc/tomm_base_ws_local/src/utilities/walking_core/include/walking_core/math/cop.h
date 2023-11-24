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

#ifndef WALKING_CORE_COP_H_
#define WALKING_CORE_COP_H_

#include <control_core/types.h>

namespace cc 
{

  /**
   * @brief convert com and zmp into equivalent net contact wrench
   * 
   * See: Stair climbing stabilization eq.: 9
   * 
   * @param com 
   * @param zmp 
   * @return cc::Wrench 
   */
  inline cc::Wrench net_contact_wrench(const cc::LinearPosition& com, const cc::LinearPosition& zmp, cc::Scalar mass)
  {
    cc::Scalar omega2 = CC_GRAVITY/com.z();
    cc::Wrench W_com = cc::Wrench::Zero();
    W_com.force() = mass*omega2*(com - zmp);
    return W_com;
  }

  /**
   * @brief compute cop foot sole frame.
   * 
   * If an additional translation exists between the Wrench's sensor frame and
   * the cop frame, it can be included in t.
   * 
   * @param W wrench in force torque sensor frame
   * @param t translation force torque sensor frame wrt foot sole frame position
   * @return cc::LinearPosition cop position in foot sole frame
   */
  inline cc::LinearPosition wrench_cop(
    const cc::Wrench& W, 
    const cc::LinearPosition& t = cc::LinearPosition::Zero())
  {
    cc::LinearPosition cop = cc::LinearPosition::Zero();
    if(W.force().z() > 1e-3)
    {
      cop.x() += (-W.moment().y() - W.force().x() * t.z() + W.force().z() * t.x()) / W.force().z();
      cop.y() += (W.moment().x() - W.force().y() * t.z() + W.force().z() * t.y()) / W.force().z();
    }
    cop.z() = 0.0;
    return cop;
  }

  /**
   * @brief computes the torque required to produce zmp p in frame with center x.
   * Toqure is expessed in translated sensor frame t
   * 
   * Note: this function should be changed and instead take input in foot sole frame
   * 
   * @param p cop in world frame
   * @param x foot sole frame center wrt world 
   * @param f force wrt world frame
   * @param t translation force torque sensor frame wrt foot sole frame position
   */
  inline cc::Moment cop_wrench(const cc::LinearPosition& p, const cc::LinearPosition& x, const cc::Force& f, const cc::LinearPosition& t = cc::LinearPosition::Zero())
  {
    return (cc::Moment()<<
      (p.y() - x.y())*f.z() + f.y()*t.z() - f.z()*t.y(),
      -(p.x() - x.x())*f.z() -f.x()*t.z() + f.z()*t.x(),
      0.0).finished();
  }

}

#endif