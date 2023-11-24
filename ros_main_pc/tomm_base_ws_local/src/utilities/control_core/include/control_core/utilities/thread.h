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

#ifndef CONTROL_CORE_THREAD_H_
#define CONTROL_CORE_THREAD_H_

#include <control_core/utilities/type_not_assignable.h>

#include <control_core/test_utilities/timing.h>

#include <thread>
#include <sys/syscall.h>
#include <sys/types.h>

/*! \file thread.h
 *  \brief Contains Thread Wrapper interface
 */

#include <iostream>

namespace cc
{

  /**
   * @brief The thread class wrappes a std::thread object.
   * 
   * This Class can be used as a base class to execute code in another thread.
   * After the start() is called, the run() function will be executed in
   * a new thread.
   */
  class Thread
  {
    OW_TYPE_NOT_ASSIGNABLE(Thread)

  protected:
    std::thread thread_;
    pthread_t native_thread_;
    bool started_;   // true if thread run() active
    bool stop_request_;   // true if thread is about to stop

  public:
    /**
     * @brief Construct a new Thread object
     * 
     */
    Thread() : started_(false)
    {
    }

    /**
     * @brief Destroy the Thread object
     * 
     */
    virtual ~Thread()
    {
      if (thread_.joinable())
      {
        thread_.join();
      }
    }

    /**
     * @brief returns the thread id
     * 
     * @return std::thread::id 
     */
    std::thread::id getID()
    {
      return thread_.get_id();
    }

    /**
     * @brief checks if the thread is stopped
     * 
     * @return true 
     * @return false 
     */
    bool isFinished()
    {
      return !started_;
    }

    /**
     * @brief checks if the thread is running
     * 
     * @return true 
     * @return false 
     */
    bool isActive()
    {
      return started_;
    }
    
    /**
     * @brief checks if the thread has a stop request
     * note: use this inside run to check when to stop
     * 
     * @return true 
     * @return false 
     */
    bool hasStopRequest()
    {
      return stop_request_;
    }

    /**
     * @brief start the thread
     * 
     * @return true 
     * @return false 
     */
    bool start()
    {
      if (!isActive())
      {
        // if there is an old thread, active, join it first!
        if (thread_.joinable())
        {
          thread_.join();
        }
        
        // create thread
        thread_ = std::thread(&Thread::runInternal, this);
        native_thread_ = thread_.native_handle();
        return true;
      }
      return false;
    }
    
    /**
     * @brief set the stop request
     * 
     * @return true 
     * @return false 
     */
    bool stop()
    {
      // set the stop flag
      stop_request_ = true;

      // wait until the thread has stopped!
      auto start = TIMENOW();
      while(started_ && DURATION(start) < 5e6)
      {
        usleep(100);
      }
      if(started_)
      {
        pthread_cancel(native_thread_);
        started_ = false;
        return false;
      }
      return true;
    }

  private:
    virtual void runInternal()
    {
      started_ = true;
      stop_request_ = false;
      run();
      started_ = false;
    }

  protected:
    /*!
    * @brief The thread execution function.
    */
    virtual void run() = 0;
  };

} // namespace cc

#endif // CONTROL_CORE_THREAD_H_