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


#ifndef WALKING_CORE_TYPES_H
#define WALKING_CORE_TYPES_H

/*! \file types.h
 *  \brief Contains all the types.
 */

// get the configurations to correctly define all types
#include <control_core/configuration.h>

#include <walking_core/types/foot_step.h>
#include <walking_core/types/walking_phase.h>
#include <walking_core/types/walking_states.h>

/*!
 * \brief The namespace for the configured types.
 */
namespace cc
{
  /*! Type for Scalars */
  typedef CC_TYPES_SCALAR Scalar;

  /*! Basic Types */
  typedef control_core::FootStep<Scalar> FootStep;
  typedef control_core::WalkingPhase WalkingPhase;
  typedef control_core::WalkingStates<Scalar> WalkingStates;
  typedef std::vector<FootStep> FootSteps;

}

#endif  // WALKING_CORE_TYPES_H
