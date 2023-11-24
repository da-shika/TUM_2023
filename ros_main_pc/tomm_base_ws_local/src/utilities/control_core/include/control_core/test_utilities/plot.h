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


#ifndef CONTROL_CORE_PLOT_UTILITIES_H
#define CONTROL_CORE_PLOT_UTILITIES_H

#include <control_core/test_utilities/matplotlibcpp.h>

namespace cc_test
{

  // for convenience
  namespace plt = matplotlibcpp;

  /**
   * @brief plot signal x, y
   * 
   * @tparam VectorX 
   * @tparam VectorY 
   * @param x 
   * @param y 
   * @param fig_name 
   * @param grid 
   * @param show 
   * @return true 
   * @return false 
   */
  template <typename VectorX, typename VectorY>
  bool plot(
    const VectorX &x, const VectorY &y, 
    const std::string& lable, const std::string& fig_name, const std::string& line_style="-",
    bool grid=true, bool show=false, long fig_idx=-1)
  {
    bool ret;
    plt::figure(fig_idx);
    plt::title(fig_name);
    ret = plt::plot(x, y, {{"label", lable}, {"linestyle", line_style}});
    plt::grid(grid);
    plt::legend();
    if(show)
      plt::show();
    return ret; 
  }

}

#endif