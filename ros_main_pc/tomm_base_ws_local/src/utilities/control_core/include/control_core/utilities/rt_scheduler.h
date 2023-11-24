#ifndef CONTROL_CORE_UTILITIES_RT_SCHEDULER_H_
#define CONTROL_CORE_UTILITIES_RT_SCHEDULER_H_

#include <pthread.h>
#include <ros/console.h>

namespace cc
{

  /**
   * @brief set priority and scheduler of current thread
   * 
   * @param priority value between 0 and 99 
   * 
   * example:
   *  const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
   *  cc::set_scheduling(max_thread_priority - 1, true);
   */
  inline bool set_scheduling(int priority, bool verbose=true)
  {
    pthread_t this_thread = pthread_self();

    if (priority != -1)
    {
      sched_param params;
      params.sched_priority = priority;

      int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
      if(ret != 0)
      {
        if(verbose)
          ROS_ERROR_STREAM("Unsuccessful in setting Communication thread realtime priority. Error code: " << ret);
        return false;
      }

      int policy = 0;
      ret = pthread_getschedparam(this_thread, &policy, &params);
      if( ret != 0)
      {
        if(verbose)
          ROS_ERROR_STREAM("Couldn't retrieve real-time scheduling paramers");
        return false;
      }

      if (policy != SCHED_FIFO)
      {
        if(verbose)
          ROS_ERROR_STREAM("Scheduling is NOT SCHED_FIFO!");
        return false;
      }
      else
      {
        if(verbose)
          ROS_INFO("SCHED_FIFO OK");
      }
      if(verbose)
        ROS_INFO_STREAM("Communication thread priority is " << params.sched_priority);
      return true;
    }

    if(verbose)
      ROS_INFO_STREAM("Could not get maximum thread priority for Communication thread");
    return false;
  }

}

#endif