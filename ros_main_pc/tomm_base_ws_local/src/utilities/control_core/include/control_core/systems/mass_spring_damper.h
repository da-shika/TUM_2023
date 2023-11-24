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

#ifndef CONTROL_CORE_SYSTEM_MASS_SPRING_DAMPER_H
#define CONTROL_CORE_SYSTEM_MASS_SPRING_DAMPER_H

#include <control_core/types.h>
#include <control_core/math.h>
#include <control_core/ros/parameters.h>

namespace cc
{  
  /**
   * @brief MassSpringDamperSystem Class
   * 
   * Simulate state of second order mass spring damper system for given
   * input wrench.
   */
  class MassSpringDamperSystem
  {
    private:
      cc::CartesianState state_;

      // impedance model
      cc::Vector6 mass_;
      cc::Vector6 spring_; 
      cc::Vector6 damping_;

      // bounds
      cc::Vector6 acc_lb_, acc_ub_;
      cc::Vector6 vel_lb_, vel_ub_;
      cc::LinearPosition pos_lb_, pos_ub_;

    public:
      MassSpringDamperSystem();
      virtual ~MassSpringDamperSystem();

      void setMass(const cc::Vector6& mass);
      void setSpring(const cc::Vector6& spring);
      void setDamping(const cc::Vector6& damping);
      void setAccBounds(const cc::Vector6& acc_lb, const cc::Vector6& acc_ub);
      void setVelBounds(const cc::Vector6& vel_lb, const cc::Vector6& vel_ub);
      void setPosBounds(const cc::LinearPosition& pos_lb, const cc::LinearPosition& pos_ub);

      const cc::Vector6& mass() { return mass_; }
      const cc::Vector6& spring() { return spring_; }
      const cc::Vector6& damping() { return damping_; }

      /**
       * @brief inialize from parameters
       * 
       * @param params 
       * @return true 
       * @return false 
       */
      bool init(cc::Parameters& params);

      /**
       * @brief reset the system to some inital pose 
       * 
       * @param inital_pos 
       */
      void reset(const cc::CartesianPosition inital_pos);

      /**
       * @brief update the system with given input wrench (in body frame)
       * 
       * @param wrench 
       * @param dt 
       */
      void update(const cc::Wrench wrench, const ros::Duration& dt);

      /**
       * @brief return the system state (pos, vel, acc)
       * 
       * @return const cc::CartesianState& 
       */
      const cc::CartesianState& state() const;
  };

}

#endif