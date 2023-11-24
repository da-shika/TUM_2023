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

#ifndef CONTROL_CORE_MATH_ADJOINT_H
#define CONTROL_CORE_MATH_ADJOINT_H

#include <control_core/math/cartesian.h>

/**
 * @brief Contains function for adjoint transformations 
 */

// the namespace for the project
namespace cc
{
  /**
   * @brief matrix representation of the cross product operation
   * 
   * @param v 
   * @return cc::Matrix3 
   */
  inline cc::Matrix3 skrew(const cc::Vector3& v)
  {
    return (cc::Matrix3()<<
      0, -v.z(), v.y(),
      v.z(), 0, -v.x(),
      -v.y(), v.x(), 0).finished(); 
  }

  /**
   * @brief adjoint transformation matrix for motion vector
   * 
   * Transforms (t,R) from frame a into b.
   * 
   * @param t translation frame a wrt to b
   * @param R rotation frame a wrt to b
   * @return cc::Matrix6 
   */
  inline cc::Matrix6 adjointMatrixMotion(const cc::LinearPosition& t, const cc::Rotation3& R)
  {
    return (cc::Matrix6()<< 
      R, skrew(t)*R,
      cc::Matrix3::Zero(), R).finished();
  }

  inline cc::Matrix6 adjointMatrixMotion(const cc::CartesianPosition& X)
  {
    return adjointMatrixMotion(X.linear(), X.angular().toRotationMatrix());
  }

  /**
   * @brief adjoint transformation matrix for wrench vector
   * 
   * Transforms (t,R) from frame a into b.
   * 
   * @param t translation frame a wrt to b
   * @param R rotation frame a wrt to b
   * @return cc::Matrix6 
   */
  inline cc::Matrix6 adjointMatrixWrench(const cc::LinearPosition& t, const cc::Rotation3& R)
  {
    return (cc::Matrix6()<< 
      R, cc::Matrix3::Zero(),
      skrew(t)*R, R).finished();
  }

  inline cc::Matrix6 adjointMatrixWrench(const cc::CartesianPosition& X)
  {
    return adjointMatrixWrench(X.linear(), X.angular().toRotationMatrix());
  }

  /**
   * @brief adjoint transformation Ad_a_b of motion V_a to V_b 
   * 
   * @param t 
   * @param R 
   * @param W 
   * @return cc::Wrench 
   */
  inline cc::CartesianVelocity motionTransformation(const cc::LinearPosition& t, const cc::Rotation3& R, const cc::CartesianVelocity& V)
  {
    return adjointMatrixMotion(t, R)*V;
  }

  inline cc::Wrench motionTransformation(const cc::CartesianPosition& X, const cc::CartesianVelocity& V)
  {
    return motionTransformation(X.linear(), X.angular().toRotationMatrix(), V);
  }

  /**
   * @brief adjoint transformation Ad_a_b of wrench W_a to W_b 
   * 
   * @param t 
   * @param R 
   * @param W 
   * @return cc::Wrench 
   */
  inline cc::Wrench wrenchTransformation(const cc::LinearPosition& t, const cc::Rotation3& R, const cc::Wrench& W)
  {
    return adjointMatrixWrench(t, R)*W;
  }

  inline cc::Wrench wrenchTransformation(const cc::CartesianPosition& X, const cc::Wrench& W)
  {
    return wrenchTransformation(X.linear(), X.angular().toRotationMatrix(), W);
  }

}

#endif