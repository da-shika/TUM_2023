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

#ifndef CONTROL_CORE_INTERFACES_GENERIC_BASE_H
#define CONTROL_CORE_INTERFACES_GENERIC_BASE_H

#include <control_core/utilities/print.h>

namespace cc 
{
  /**
   * @brief GenericBase Class
   * 
   * GenericBase keeps track of class names.
   * Every class should derive from this.
   * 
   */
  class GenericBase
  {
  private:
    std::string name_;        //!< the module name

  public:
    /*!
    * \brief Constructor.
    */
    GenericBase(const std::string& name) : 
      name_(name)
    {
    }

    /*!
    * \brief Virtual destructor.
    */
    virtual ~GenericBase()
    {
    }

    /** 
     * \brief get state name
     */
    const std::string& name() const
    { 
      return name_;
    }

    /** 
     * \brief get state name
     */
    std::string classType() const
    {
      return name_;
    }
    
  };
}

#endif
