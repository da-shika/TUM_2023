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

#ifndef WALKING_CORE_WALKING_STATE_H_
#define WALKING_CORE_WALKING_STATE_H_

#include <control_core/types/body_id.h>
#include <walking_core/types/foot_step.h>

#include <ros/ros.h>

#include <walking_core_msgs/WalkingStates.h>

namespace control_core 
{

  /*!
  * \brief The WalkingStates class.
  *
  * This class is a container for the WalkingStates.
  * It contain the current walking phase, elapsed times, planned footsteps
  * 
  */
  template<typename _Scalar>
  class WalkingStates
  {
  public:
    typedef _Scalar Scalar;
    typedef FootStep<Scalar> Step;

  protected:
    // current states
    WalkingPhase phase_;
    control_core::BodyId stance_foot_;              //!< support foot according to plan
    control_core::BodyId swing_foot_;               //!< swing foot according to plan
    Scalar elapsed_;                                //!< elapsed phase start(0), end(1)
    Scalar left_foot_ratio_;                        //!< weight on left (0) or right foot (1)

    Step cur_step_;                                 //!< current support foot (on the ground)
    Step target_step_;                              //!< swing target
    Step next_step_;                                //!< next swing target

    cc::CartesianPosition left_foot_ref_;           //!< left foot ref position
    cc::CartesianPosition right_foot_ref_;          //!< right foot ref position

  public:
    WalkingStates() : 
      phase_(WalkingPhase::STANCE),
      stance_foot_(control_core::BodyId::LEFT_FOOT),
      swing_foot_(control_core::BodyId::RIGHT_FOOT),
      left_foot_ratio_(0.5),
      elapsed_(0.0)
    {
    }

    /*!
   * \brief Copy constructor.
   */
    WalkingStates(const WalkingStates& other) : 
      phase_(other.phase_),
      stance_foot_(other.stance_foot_),
      swing_foot_(other.swing_foot_),
      left_foot_ratio_(other.left_foot_ratio_),
      elapsed_(other.elapsed_),
      cur_step_(other.cur_step_),
      target_step_(other.target_step_),
      next_step_(other.next_step_),
      left_foot_ref_{other.left_foot_ref_},
      right_foot_ref_{other.right_foot_ref_}
    {
    }

    WalkingStates& operator=(const WalkingStates& other)
    {
      phase_ = other.phase_;
      stance_foot_ = other.stance_foot_;
      swing_foot_ = other.swing_foot_;
      left_foot_ratio_ = other.left_foot_ratio_;
      elapsed_ = other.elapsed_;
      cur_step_ = other.cur_step_;
      target_step_ = other.target_step_;
      next_step_ = other.next_step_;
      left_foot_ref_ = other.left_foot_ref_;
      right_foot_ref_ = other.right_foot_ref_;
      return *this;
    }

    WalkingStates& operator=(const walking_core_msgs::WalkingStates& msg)
    {
      phase_ = msg.phase;
      stance_foot_ = static_cast<control_core::BodyId>(msg.stance_foot.id);
      swing_foot_ = static_cast<control_core::BodyId>(msg.swing_foot.id);
      left_foot_ratio_ = msg.left_foot_ratio.data;
      elapsed_ = msg.elapsed.data;
      cur_step_ = msg.cur_step;
      target_step_ = msg.target_step;
      next_step_ = msg.next_step;
      return *this;
    }

    operator walking_core_msgs::WalkingStates() const
    {
      walking_core_msgs::WalkingStates msg;
      msg.phase = phase_;
      msg.stance_foot.id = stance_foot_;
      msg.swing_foot.id = swing_foot_;
      msg.left_foot_ratio.data = left_foot_ratio_;
      msg.elapsed.data = elapsed_;
      msg.cur_step = cur_step_;
      msg.target_step = target_step_;
      msg.next_step = next_step_;
      return msg;
    } 

    WalkingPhase& phase()
    {
      return phase_;
    }

    const WalkingPhase& phase() const
    {
      return phase_;
    }

    Scalar& leftFootRatio()
    {
      return left_foot_ratio_;
    }

    Scalar leftFootRatio() const
    {
      return left_foot_ratio_;
    }
    
    control_core::BodyId& swingFoot()
    {
      return swing_foot_;
    }

    const control_core::BodyId& swingFoot() const
    {
      return swing_foot_;
    }
    
    control_core::BodyId& stanceFoot()
    {
      return stance_foot_;
    }

    const control_core::BodyId& stanceFoot() const
    {
      return stance_foot_;
    }

    Scalar& elapsed()
    {
      return elapsed_;
    }

    Scalar elapsed() const
    {
      return elapsed_;
    }

    Step& curStep()
    {
      return cur_step_;
    }

    const Step& curStep() const
    {
      return cur_step_;
    }

    Step& targetStep()
    {
      return target_step_;
    }

    const Step& targetStep() const
    {
      return target_step_;
    }

    Step& nextStep()
    {
      return next_step_;
    }

    const Step& nextStep() const
    {
      return next_step_;
    }

    cc::CartesianPosition& leftFootRef()
    {
      return left_foot_ref_;
    }

    const cc::CartesianPosition& leftFootRef() const
    {
      return left_foot_ref_;
    }

    cc::CartesianPosition& rightFootRef()
    {
      return right_foot_ref_;
    }

    const cc::CartesianPosition& rightFootRef() const
    {
      return right_foot_ref_;
    }

    std::string toString() const
    {
      std::stringstream out;
      out << "phase  :          " << phase_.toString() << "\n";
      out << "stance_foot  :    " << stance_foot_.toString() << "\n";
      out << "swing_foot    :   " << swing_foot_.toString() << "\n";             
      out << "left_foot_ratio:  " << left_foot_ratio_ << "\n";
      out << "elapsed:          " << elapsed_ << "\n";
      out << "left_foot_ref:    " << left_foot_ref_.toString() << "\n";
      out << "right_foot_ref:   " << right_foot_ref_.toString() << "\n";
      return out.str();
    }

  };

} // namespace control_core


#endif