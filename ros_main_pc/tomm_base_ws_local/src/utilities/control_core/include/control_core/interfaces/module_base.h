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

#ifndef CONTROL_CORE_INTERFACES_MODULE_BASE_H
#define CONTROL_CORE_INTERFACES_MODULE_BASE_H

#include <control_core/interfaces/generic_module_base.h>

#include <control_core/ros/parameters.h>
#include <control_core/utilities/print.h>

namespace cc 
{
  /**
   * @brief ModuleBase Class
   * 
   * ModuleBase is base for all modules. Keeps track of module state and 
   * stores start and stop time.
   * 
   * Requires overloading: 
   *  init()
   * Optional overloading:
   *  start()
   *  update()
   *  stop()
   *  publish()
   * 
   */
  class ModuleBase : public GenericModuleBase
  {
  public:
    typedef cc::GenericModuleBase Base;
    typedef cc::Parameters Parameters;

  private:
    bool has_updated_once_;       //!< remember if updated once  

  public:
    /*!
    * \brief Constructor.
    */
    ModuleBase(const std::string& name) : 
      Base(name),
      has_updated_once_(false)
    {
    }

    /*!
    * \brief Virtual destructor.
    */
    virtual ~ModuleBase()
    {
    }

    /**
     * @brief start the Controller
     * 
     * @param time 
     * @return true 
     * @return false 
     */
    virtual bool startRequest(const ros::Time &time)
    {
      if (!Base::moduleSetStartState(ros::Time::now()))
      {
        PRINT_ERROR("'%s' called but not initalized.", Base::name().c_str());
        return false;
      }
      start(time);
      has_updated_once_ = false;
      return true;
    }

    /**
     * @brief stop the Controller
     *
     * @param time 
     * @return true 
     * @return false 
     */
    virtual bool stopRequest(const ros::Time &time)
    {
      if (Base::moduleSetStopState(time))
      {
        stop(time);
        return true;
      }
      return false;
    }

    /**
     * @brief update
     * 
     * @param time 
     * @param period 
     * @return true 
     * @return false 
     */
    virtual bool updateRequest(
        const ros::Time &time,
        const ros::Duration &period)
    {
      if (!isRunning())
      {
        PRINT_ERROR("'%s' called but not running.", Base::name().c_str());
        return false;
      }
      bool ret = update(time, period);
      has_updated_once_ = true;
      return ret;
    }

    /**
     * @brief publish information
     * 
     * @param time 
     */
    virtual void publishRequest(const ros::Time &time)
    {
      if(has_updated_once_)
      {
        publish(time);
      }
    }

  protected:
    /**
     * @brief start the controller
     * 
     * @param time 
     */
    virtual void start(const ros::Time &time)
    {
      // optional
    }

    /**
     * @brief update the controller
     * 
     * @param time 
     * @param period 
     * @return true 
     * @return false 
     */
    virtual bool update(const ros::Time &time, const ros::Duration &period)
    {
      // optional
      return true;
    }

    /**
     * @brief stop the controller
     * 
     * @param time 
     */
    virtual void stop(const ros::Time &time)
    {
      // optional
    }

    /**
     * @brief publish information
     * 
     * @param time 
     */
    virtual void publish(const ros::Time &time)
    {
      // optional
    }
  };

}

#endif
