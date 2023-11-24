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


#ifndef CONTROL_CORE_TYPES_H
#define CONTROL_CORE_TYPES_H

/*! \file types.h
 *  \brief Contains all the types.
 */

#include <control_core/primitive_types.h>

#include <control_core/types/body_id.h>

#include <control_core/types/robot_state.h>
#include <control_core/types/imu_sensor.h>
#include <control_core/types/ft_sensor.h>
#include <control_core/types/skin_modality.h>
#include <control_core/types/skin_patch.h>
#include <control_core/types/contact.h>

/*!
 * \brief The namespace for the configured types.
 */
namespace cc
{
  /*! Type for Scalars */
  typedef CC_TYPES_SCALAR Scalar;

  /* Sensor Types */
  typedef control_core::BodyId BodyId;
  typedef control_core::ImuSensor<Scalar> ImuSensor;
  typedef control_core::FtSensor<Scalar> FtSensor;
  typedef control_core::SkinModality<Scalar> SkinModality;
  typedef control_core::SkinPatch<Scalar> SkinPatch;
  typedef control_core::RobotState<Scalar> RobotState;

  typedef std::vector<FtSensor> FtSensors;
  typedef std::vector<SkinPatch> SkinPatches;

  /* Contacts */
  typedef control_core::Contact<Scalar> Contact;
  typedef std::vector<Contact> Contacts;
}

#endif  // CONTROL_CORE_TYPES_H
