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

#ifndef CONTROL_CORE_MATH_SHORTEST_PATH_H
#define CONTROL_CORE_MATH_SHORTEST_PATH_H

#include <control_core/math/quaternion.h>

namespace cc
{

  /**
   * @brief Modify the desired Quaternion to yield shortest rotation
   * 
   * @tparam Derived1 
   * @tparam Derived2 
   * @param Qd Desired Quaternion
   * @param Q Quaternion
   * @return cc::AngularPosition of the modified desired Quaternion
   */
  template <typename Derived1, typename Derived2>
  inline cc::AngularPosition shortestPath(
      const control_core::QuaternionBase<Derived1> &Qd,
      const control_core::QuaternionBase<Derived2> &Q)
  {
    cc::AngularPosition Qd_mod = Qd.derived() * Q.derived().inverse();
    checkFlipQuaternionSign(Qd_mod);
    return Qd_mod * Q.derived();
  }

  /**
   * @brief Modify the desired CartesianPosition to yield shortest movement
   * 
   * @param Xd 
   * @param X 
   * @return cc::CartesianPosition 
   */
  inline cc::CartesianPosition shortestPath(
      const cc::CartesianPosition &Xd,
      const cc::CartesianPosition &X)
  {
    cc::CartesianPosition Xd_mod;
    Xd_mod.linear() = Xd.linear();
    Xd_mod.angular() = shortestPath(Xd.angular(), X.angular());
    return Xd_mod;
  };

  /**
   * @brief Modify the desired vector to yield shortest movement
   * 
   * @note This function is only used to handle the general case in template class
   * 
   * @param Xd 
   * @param X 
   * @return cc::CartesianPosition 
   */
  template <typename Derived1, typename Derived2>
  inline Eigen::Matrix<typename Derived1::Scalar, Derived1::RowsAtCompileTime, Derived1::ColsAtCompileTime>
  shortestPath(
      const Eigen::MatrixBase<Derived1> &xd,
      const Eigen::MatrixBase<Derived2> &x)
  {
    return xd;
  };

} // namespace cc

#endif
