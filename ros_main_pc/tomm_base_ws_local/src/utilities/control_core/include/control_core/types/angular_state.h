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


#ifndef CONTROL_CORE_ANGULAR_STATE_H
#define CONTROL_CORE_ANGULAR_STATE_H

#include <control_core/types/angular_position.h>
#include <control_core/types/angular_velocity.h>
#include <control_core/types/angular_acceleration.h>

#include <control_core/type_bases/state_base.h>

#include <control_core_msgs/AngularState.h>

namespace cc{

/*!
 * \brief The traits class for the AngularState class.
 *
 * This class contains the typedefs and enums for
 * the AngularState class.
 *
 * These typedefs and enums are used to define and describe
 * the properties (traits) of this class.
 */
template<typename _Scalar>
struct traits<control_core::AngularState<_Scalar> >
{
  typedef _Scalar Scalar;
  typedef control_core::AngularPosition<Scalar> Pos;
  typedef control_core::AngularVelocity<Scalar> Vel;
  typedef control_core::AngularAcceleration<Scalar> Acc;
  
  enum
  {
    IsRef = 0,
  };
};

} // namespace cc


namespace control_core{

/*!
 * \brief The AngularState class.
 *
 *  This class is a container for:
 *    - AngularPosition
 *    - AngularVelocity
 *    - AngularAcceleration
 *    - Moment
 */
template<typename _Scalar>
class AngularState : 
  public StateBase<AngularState<_Scalar> >
{
public:
  typedef _Scalar Scalar;
  typedef StateBase<AngularState<Scalar> > Base;

  typedef AngularPosition<Scalar> Pos;
  typedef AngularVelocity<Scalar> Vel;
  typedef AngularAcceleration<Scalar> Acc;

protected:
  Pos Q_;
  Vel omega_;
  Acc alpha_;

public:
  /*!
    * \brief Default Constructor.
    */
  AngularState()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  AngularState(const AngularState& other) :
    Q_(other.Q_),
    omega_(other.omega_),
    alpha_(other.alpha_)
  {
  }

  /*!
   * \brief Copy constructor.
   */
  AngularState(const Base& other) : 
    Q_(other.pos()),
    omega_(other.vel()),
    alpha_(other.acc())
  {
  }

  /*!
    * \brief Constructor from sub elements
    */
  explicit AngularState(const Pos& Q, const Vel& omega = Vel::Zero(), const Acc& alpha = Acc::Zero()) : 
    Q_(Q),
    omega_(omega),
    alpha_(alpha)
  {
  }

  /*!
    * \brief access to orientation part.
    */
  Pos& Q()
  {
    return Q_;
  }

  /*!
    * \brief access to orientation part.
    */
  Pos& pos()
  {
    return Q_;
  }

  /*!
    * \brief access to orientation part.
    */
  const Pos& Q() const
  {
    return Q_;
  }

  /*!
    * \brief access to orientation part.
    */
  const Pos& pos() const
  {
    return Q_;
  }

  /*!
    * \brief access to velocity part.
    */
  Vel& omega()
  {
    return omega_;
  }

  /*!
    * \brief access to velocity part.
    */
  Vel& vel()
  {
    return omega_;
  }

  /*!
    * \brief access to velocity part.
    */
  const Vel& omega() const
  {
    return omega_;
  }

  /*!
    * \brief access to velocity part.
    */
  const Vel& vel() const
  {
    return omega_;
  }

  /*!
    * \brief access to acceleration part.
    */
  Acc& alpha()
  {
    return alpha_;
  }

  /*!
    * \brief access to acceleration part.
    */
  Acc& acc()
  {
    return alpha_;
  }

  /*!
    * \brief access to acceleration part.
    */
  const Acc& alpha() const
  {
    return alpha_;
  }

  /*!
    * \brief access to acceleration part.
    */
  const Acc& acc() const
  {
    return alpha_;
  }

  /*!
   * \brief Use assignment operators of base class.
   */
  using Base::operator=;

  /*!
   * \brief Assignment operator.
   */
  AngularState& operator=(const AngularState& other)
  {
    Base::operator=(other);
    return *this;
  }

  /*!
   * \brief Assignment of control_core_msgs::AngularState.
   */
  AngularState& operator=(const control_core_msgs::AngularState& msg)
  {
    pos() = msg.position;
    vel() = msg.velocity;
    acc() = msg.acceleration;
    return *this;
  }

  /*!
   * \brief Conversion to control_core_msgs::AngularState.
   */
  operator control_core_msgs::AngularState() const
  {
    control_core_msgs::AngularState msg;
    msg.position = pos();
    msg.velocity = vel();
    msg.acceleration = acc();
    return msg;
  }  

  /*!
   * \brief Conversion to control_core_msgs::AngularState.
   */
  control_core_msgs::AngularState toAngularStateMsg() const
  {
    return static_cast<control_core_msgs::AngularState>(*this);
  }

};

} // namespace control_core

#endif // CONTROL_CORE_ANGULAR_STATE_H
