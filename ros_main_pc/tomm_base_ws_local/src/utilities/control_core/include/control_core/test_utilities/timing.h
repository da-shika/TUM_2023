#ifndef CONTROL_CORE_TEST_UTILITES_TIMING_H_
#define CONTROL_CORE_TEST_UTILITES_TIMING_H_

#include <chrono>
#include <utility>

typedef std::chrono::high_resolution_clock::time_point TimeVar;

/**
 * @brief get current time
 */
#define TIMENOW() std::chrono::high_resolution_clock::now()

/**
 * @brief get durtation since start in ns
 * 
 */
#define DURATION(start) std::chrono::duration_cast<std::chrono::nanoseconds>(TIMENOW() - start).count()

namespace cc
{

  /**
   * @brief measure elapsed time of func in ns
   */
  template<typename F, typename... Args>
  inline double func_time(F func, Args&&... args)
  {
    TimeVar start = TIMENOW();
    func(std::forward<Args>(args)...);
    return DURATION(start);
  }

}

#endif
