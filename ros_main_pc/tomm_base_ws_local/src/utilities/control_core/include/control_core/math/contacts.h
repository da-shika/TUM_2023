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

#ifndef CONTROL_CORE_MATH_CONTACTS_H
#define CONTROL_CORE_MATH_CONTACTS_H

#include <control_core/types.h>

// the namespace for the project
namespace cc
{

  /**
   * @brief returns the H representation of point contact polytrope
   * 
   * A stable Force fullfiles the inequality H*f < 0
   * 
   * @param mu 
   * @return Eigen::Matrix<Scalar,4,3> 
   */
  inline Eigen::Matrix<Scalar,4,3> pointContactHRep(Scalar mu)
  {
    return (Eigen::Matrix<Scalar,4,3>() << 
      -1,  0, -mu,
      +1,  0, -mu,
       0, -1, -mu,
       0, +1, -mu).finished();
  }

  /**
   * @brief returns the V respresentation of point contact polytrope
   * 
   * A stable Force is an element of the positive span(V)
   * 
   * @param mu 
   * @return Eigen::Matrix<Scalar,4,3> 
   */
  inline Eigen::Matrix<Scalar,4,3> pointContactVRep(Scalar mu, Scalar scaling=1.0)
  {
    return scaling*(Eigen::Matrix<Scalar,4,3>()<< 
      +mu, +mu, +1, 
      +mu, -mu, +1, 
      -mu, +mu, +1, 
      -mu, -mu, +1).finished();
  }

  inline Eigen::Matrix<Scalar,5,3> pointContactVertices(
    Scalar mu, const Eigen::Matrix<Scalar,3,1>& p, Scalar scaling=1.0)
  {
    return (Eigen::Matrix<Scalar,5,3>()<< 
      pointContactVRep(mu, scaling).rowwise() + p.transpose(),
      p.transpose()).finished();
  }

  /**
   * @brief returns the Graspmatrix for a single point contact
   * 
   * @param p 
   * @return Eigen::Matrix<Scalar,6,3> 
   */
  inline Eigen::Matrix<Scalar,6,3> pointContactGraspMatrix(
    const Eigen::Matrix<Scalar,3,1>& p)
  {
    return (Eigen::Matrix<Scalar,6,3>()<<
      Eigen::Matrix<Scalar,3,3>::Identity(),
      (Eigen::Matrix<Scalar,3,3>()<<
            0, -p[2],  p[1],
         p[2],     0, -p[0],
        -p[1],  p[0],     0).finished()
      ).finished();
  }

  /**
   * @brief returns the H representation of rectangular contact polytrope
   * 
   * A stable Wrench fullfiles the inequality H*W < 0
   * 
   * @param mu 
   * @param lx half extend X direction
   * @param ly half extend Y direction
   * @return Eigen::Matrix<Scalar,16,6> 
   */
  inline Eigen::Matrix<Scalar,16,6> rectangularContactHRep(
    Scalar mu, Scalar lx, Scalar ly)
  {
    return (Eigen::Matrix<Scalar,16,6>()<<
       -1,   0,              -mu,  0,   0,   0,
       +1,   0,              -mu,  0,   0,   0,
        0,  -1,              -mu,  0,   0,   0,
        0,  +1,              -mu,  0,   0,   0,
        0,   0,              -ly, -1,   0,   0,
        0,   0,              -ly, +1,   0,   0,
        0,   0,              -lx,  0,  -1,   0,
        0,   0,              -lx,  0,  +1,   0,
      -ly,  -lx, -(lx + ly) * mu, +mu, +mu,  -1,
      -ly,  +lx, -(lx + ly) * mu, +mu, -mu,  -1,
      +ly,  -lx, -(lx + ly) * mu, -mu, +mu,  -1,
      +ly,  +lx, -(lx + ly) * mu, -mu, -mu,  -1,
      +ly,  +lx, -(lx + ly) * mu, +mu, +mu,  +1,
      +ly,  -lx, -(lx + ly) * mu, +mu, -mu,  +1,
      -ly,  +lx, -(lx + ly) * mu, -mu, +mu,  +1,
      -ly,  -lx, -(lx + ly) * mu, -mu, -mu,  +1).finished(); 
  }

}

#endif