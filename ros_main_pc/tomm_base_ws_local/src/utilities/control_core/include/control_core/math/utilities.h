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

#ifndef CONTROL_CORE_MATH_UTILITIES_H
#define CONTROL_CORE_MATH_UTILITIES_H

#include <math.h>
#include <control_core/types.h>

// the namespace for the project
namespace cc
{

  /*!
  * \brief Compute the factorial of x.
  *
  * \returns \f$\mathScalar{x}!\f$
  */
  inline Scalar factorial(Scalar x)
  {
    return tgamma(x + 1.0);
  }

  /*!
  * \brief Get the sign of x.
  *
  * \returns sign(x)
  */
  inline int sign(Scalar x)
  {
    return (Scalar(0) < x) - (x < Scalar(0));
  }

  /*!
  * \brief Convert degree to radian
  *
  * \returns rad
  */
  inline Scalar deg2rad(Scalar deg)
  {
    static const Scalar fac = M_PI / Scalar(180.0);
    return fac * deg;
  }

  /*!
  * \brief Convert radian to degree
  *
  * \returns rad
  */
  inline Scalar rad2deg(Scalar rad)
  {
    static const Scalar fac = Scalar(180.0) / M_PI;
    return fac * rad;
  }

  /**
   * @brief clamp scalar val given range 
   */
  inline Scalar clamp(Scalar val, Scalar lower, Scalar upper)
  {
    return std::min(upper, std::max(lower, val));
  }

  /*!
  * \brief Clamp Matrix val into given range [lower, upper].
  */
  template <typename Derived>
  void clamp(
      Eigen::MatrixBase<Derived> &val,
      typename Eigen::MatrixBase<Derived>::Scalar lower,
      typename Eigen::MatrixBase<Derived>::Scalar upper)
  {
    val = val.cwiseMin(upper).cwiseMax(lower);
  }

  /*!
  * \brief Clamp Matrix val into given range [lower, upper].
  */
  template <typename Derived1, typename Derived2, typename Derived3>
  void clamp(
      Eigen::MatrixBase<Derived1> &val,
      const Eigen::MatrixBase<Derived2> &lower,
      const Eigen::MatrixBase<Derived3> &upper)
  {
    val = val.cwiseMin(upper).cwiseMax(lower);
  }

  /**
   * @brief shortest signed difference between two angles
   * 
   * @param theta_goal 
   * @param theta 
   * @return double 
   */
  inline double shortest_angle_diff(double theta_goal, double theta)
  {
    return fmod(fabs(theta_goal - theta) + M_PI, 2*M_PI) - M_PI;
  }

  /*!
  * \brief convert cartesian to polar coordinates
  */
  inline double cart_to_polar(double y, double x)
  {
    return std::atan2(y, x);
  }

  /*!
  * \brief convert polar coordinates to cartesian
  */
  inline cc::Vector2 polar_to_cart(double theta)
  {
    return (cc::Vector2()<< std::cos(theta), std::sin(theta)).finished();
  }

  /**
   * @brief tanh activation function
   * 
   * smooth value between [0, v_max] with slope determined by alpha parameter
   * 
   */
  inline cc::Scalar tanhActivation(cc::Scalar time, cc::Scalar v_max, cc::Scalar alpha)
  {
    return 0.5 * v_max * (1 + std::tanh(alpha * time - M_PI));
  }

  /**
   * @brief polynomial activation function
   * 
   * smooth value between [y_min, y_max], based on input of range [x_min, x_max]
   */
  inline cc::Scalar polyActivation(cc::Scalar x, cc::Scalar x_min, cc::Scalar x_max, cc::Scalar y_min=0.0, cc::Scalar y_max=0.0)
  {
    cc::Scalar t = clamp((x-x_min)/(x_max - x_min), 0.0, 1.0);
    cc::Scalar s = -2.0*t*t*t + 3.0*t*t;
    return (y_max-y_min)*s + y_min;
  }

}

#endif