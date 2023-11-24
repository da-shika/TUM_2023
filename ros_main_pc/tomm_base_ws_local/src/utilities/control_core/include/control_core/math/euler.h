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

#ifndef CONTROL_CORE_MATH_EULER_H
#define CONTROL_CORE_MATH_EULER_H

#include <Eigen/Geometry>

// the namespace for the project
namespace cc
{

  /*template <typename Derived>
  inline Eigen::Matrix<typename Derived::Scalar,3,1>
    eulerYPR(const Eigen::QuaternionBase<Derived>& Q)
  {
    typedef typename Derived::Scalar Scalar;
    Eigen::Matrix<typename Derived::Scalar,3,1> ypr;

    ypr[0] = std::atan2(-2*Q.x()*Q.y() + 2*Q.w()*Q.z(), 
              Q.x()*Q.x() + Q.w()*Q.w() - Q.z()*Q.z() - Q.y()*Q.y());
    ypr[1] = std::asin(2*Q.x()*Q.z() + 2*Q.w()*Q.y());
    ypr[2] = std::atan2(-2*Q.y()*Q.z() + 2*Q.w()*Q.x(), 
              Q.z()*Q.z() - Q.y()*Q.y() - Q.x()*Q.x() + Q.w()*Q.w());

    return ypr;
  }*/

  template <typename Derived>
  inline Eigen::Matrix<typename Derived::Scalar,3,1>
    eulerYPR(const Eigen::MatrixBase<Derived>& R,  unsigned int solution_number = 1)
  {
    typedef typename Derived::Scalar Scalar;
    Eigen::Matrix<typename Derived::Scalar,3,1> ypr;

    struct Euler
    {
      Scalar yaw;
      Scalar pitch;
      Scalar roll;
    };

    Euler euler_out;
    Euler euler_out2;

    // Check that pitch is not at a singularity
    if (std::fabs(R.row(2).x()) >= 1)
    {
      euler_out.yaw = 0;
      euler_out2.yaw = 0;

      // From difference of angles formula
      if (R.row(2).x() < 0) // gimbal locked down
      {
        Scalar delta = std::atan2(R.row(0).y(), R.row(0).z());
        euler_out.pitch = M_PI / Scalar(2.0);
        euler_out2.pitch = M_PI / Scalar(2.0);
        euler_out.roll = delta;
        euler_out2.roll = delta;
      }
      else // gimbal locked up
      {
        Scalar delta = std::atan2(-R.row(0).y(), -R.row(0).z());
        euler_out.pitch = -M_PI / Scalar(2.0);
        euler_out2.pitch = -M_PI / Scalar(2.0);
        euler_out.roll = delta;
        euler_out2.roll = delta;
      }
    }
    else
    {
      euler_out.pitch = -std::asin(R.row(2).x());
      euler_out2.pitch = M_PI - euler_out.pitch;

      euler_out.roll = std::atan2(R.row(2).y() / std::cos(euler_out.pitch),
                               R.row(2).z() / std::cos(euler_out.pitch));
      euler_out2.roll = std::atan2(R.row(2).y() / std::cos(euler_out2.pitch),
                                R.row(2).z() / std::cos(euler_out2.pitch));

      euler_out.yaw = std::atan2(R.row(1).x() / std::cos(euler_out.pitch),
                              R.row(0).x() / std::cos(euler_out.pitch));
      euler_out2.yaw = std::atan2(R.row(1).x() / std::cos(euler_out2.pitch),
                               R.row(0).x() / std::cos(euler_out2.pitch));
    }

    if (solution_number == 1)
    {
      ypr << euler_out.yaw, euler_out.pitch, euler_out.roll;
    }
    else
    {
      ypr << euler_out2.yaw, euler_out2.pitch, euler_out2.roll;
    }
    return ypr;
  }

} // namespace cc

#endif
