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

#ifndef CONTROL_CORE_INTERFACES_GENERIC_MODULE_BASE_H
#define CONTROL_CORE_INTERFACES_GENERIC_MODULE_BASE_H

#include <control_core/interfaces/generic_base.h>

#include <control_core/ros/parameters.h>
#include <control_core/utilities/print.h>

namespace cc 
{
  /**
   * @brief GenericModuleBase Class
   * 
   * GenericModuleBase keeps track of Module state
   * state={CONSTRUCTED, INITIALIZED, RUNNING}.
   * 
   * Stores start and stop time.
   * 
   * Requires overloading: 
   *  init()
   * 
   * Every GenericModule should derive from this
   */
  class GenericModuleBase : public GenericBase
  {
  public:
    typedef cc::GenericBase Base;
    typedef cc::Parameters Parameters;

  public:
    /*!
    * \brief Current state of the module.
    */
    enum ModuleState {CONSTRUCTED, INITIALIZED, RUNNING};

  private:
    ModuleState module_state_;    //!< the current module state
    ros::Time start_time_;        //!< start time of the module
    ros::Time stop_time_;         //!< stop time of the module

  public:
    /*!
    * \brief Constructor.
    */
    GenericModuleBase(const std::string& name) : 
      Base(name),
      module_state_(CONSTRUCTED)
    {
    }

    /*!
    * \brief Virtual destructor.
    */
    virtual ~GenericModuleBase()
    {
    }

    /** 
    * \brief return if is running
    *
    * \return running flag
    */
    bool isInitialized() const 
    { 
      return module_state_ == INITIALIZED; 
    }

    /** 
    * \brief return if is running
    *
    * \return running flag
    */
    bool isRunning() const 
    { 
      return module_state_ == RUNNING; 
    }

    /**
     * \brief get start time
     */
    ros::Time startTime() const 
    {
      return start_time_;
    }

    /**
     * \brief get stop time
     */
    ros::Time stopTime() const 
    {
      return stop_time_;
    }

    /** 
    * \brief initialize the Module after construction. 
    * 
    * Parameter holds the parameters requried for initialization.
    *
    * \param ros nh for private namespace
    * \param params parameters of the module
    */
    virtual bool initRequest(ros::NodeHandle& nh, Parameters& params) 
    {
      // check if already ready
      if(module_state_ == INITIALIZED)
        return true;

      // intialize module
      if(init(nh, params)) 
      {
        module_state_ = INITIALIZED;
        return true;
      }
      return false;
    }

  protected:
    /** 
    * \brief initialize the Module after construction. 
    * 
    * This function needs to be overloaded by derived class
    *
    * \param ros nh for namespace
    */
    virtual bool init(ros::NodeHandle& nh, Parameters& params) = 0;

  protected:
    /** 
    * \brief start the module
    *
    * \param starting time
    */
    bool moduleSetStartState(const ros::Time& time) 
    {
      if(module_state_ == INITIALIZED) 
      {
        module_state_ = RUNNING;
        resetStartTime(time);
        return true;
      }
      return false;
    }

    /** 
    * \brief stop the module
    *
    * \param stopping time
    */
    bool moduleSetStopState(const ros::Time& time) 
    {
      if(module_state_ == RUNNING) 
      {
        resetStopTime(time);
        module_state_ = INITIALIZED;
        return true;
      }
      return false;
    }

  private:
    /**
     * @brief reset start time
     * 
     * @param time 
     */
    void resetStartTime(const ros::Time& time)
    {
      start_time_ = time;
    }

    /**
     * @brief reset stop time
     * 
     * @param time 
     */
    void resetStopTime(const ros::Time& time)
    {
      stop_time_ = time;
    }

  };

}

#endif
