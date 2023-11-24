#include <control_core/types.h>

namespace ics
{

  /**
   * @brief TaskWeightSpline structure
   * 
   * Store all information to spline between task
   * 
   */
  struct TaskWeightSpline
  {
    cc::Scalar elapsed;
    cc::Scalar duration;
    cc::Scalar start_weight;
    cc::Scalar goal_weight;
  };

}