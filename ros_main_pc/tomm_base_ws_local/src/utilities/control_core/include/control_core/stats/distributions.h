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

#ifndef CONTROL_CORE_DISTRIBUTIONS_H
#define CONTROL_CORE_DISTRIBUTIONS_H

#include <control_core/configuration.h>
#include <control_core/primitive_types.h>

namespace cc
{
  /**
   * @brief evaluate normal prob density function at x
   * 
   * @param mu 
   * @param std_dev 
   * @param x 
   * @return Scalar 
   */
  inline Scalar normal_pdf(const Scalar &mu, const Scalar &std_dev, const Scalar &x)
  {
    Scalar y = (x - mu) / std_dev;
    return std::exp(-0.5*y*y) / (std_dev*CC_SQRT_TWO_PI);
  }

  inline Scalar normal_log_pdf(const Scalar &mu, const Scalar &std_dev, const Scalar &x)
  {
    Scalar y = (x - mu) / std_dev;
    return -std::log(std_dev*CC_SQRT_TWO_PI) - 0.5*y*y;
  }

  /**
   * @brief evaulate cumulated density function of normal distribution at x
   *
   * @param x
   * @return Scalar
   */
  inline Scalar normal_cdf(const Scalar &mu, const Scalar &std_dev, const Scalar &x)
  {
    return 0.5 * std::erfc(-(x - mu) / (std_dev * M_SQRT1_2));
  }

}

#endif