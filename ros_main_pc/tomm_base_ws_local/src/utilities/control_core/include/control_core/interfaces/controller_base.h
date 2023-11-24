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

#ifndef CONTROL_CORE_INTERFACES_CONTROLLER_BASE_H_
#define CONTROL_CORE_INTERFACES_CONTROLLER_BASE_H_

#include <control_core/interfaces/module_base.h>

namespace cc 
{
  /**
   * @brief Generic Module base Class
   * 
   * Generic ModuleBase keeps track of Module
   * state={CONSTRUCTED, INITIALIZED, RUNNING}.
   * 
   * Stores start and stop time.
   * 
   * Requires: 
   *  init()
   *  command()
   * Optional:
   *  start()
   *  update()
   *  stop()
   * 
   */
  class ControllerBase : public ModuleBase
  {
  public:
    typedef ModuleBase Base;
    typedef cc::Parameters Parameters;
  
  public:
    /*!
    * \brief Constructor.
    */
    ControllerBase(const std::string& name) : 
      Base(name)
    {
    }

    /*!
    * \brief Virtual destructor.
    */
    virtual ~ControllerBase()
    {
    }

    /**
     * @brief obtain the latest control command
     * 
     * @return const cc::JointState 
     */
    virtual const cc::JointState& command() const = 0;
  };

}

#endif