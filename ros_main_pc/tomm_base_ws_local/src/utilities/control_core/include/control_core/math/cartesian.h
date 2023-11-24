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

#ifndef CONTROL_CORE_MATH_CARTESIAN_H
#define CONTROL_CORE_MATH_CARTESIAN_H

#include <control_core/math/quaternion.h>

/**
 * @brief Contains function for CartesianPosition and CartesianState.
 * 
 * @note Linear Velocites are represented in world frame. Angular Velocites can
 * be represented in world or body frame.
 */

// the namespace for the project
namespace cc
{
  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired CartesianPosition Xd_b_w body wrt world
  * and the CartesianPosition X_b_w expressed in world.
  * 
  * @note The resulting 6d error expresses linear velocity in world frame
  * and angular velocity in body world frame
  * 
  * \return cartesian error w.r.t world
  * e_w = cartesianError(Xd_b_w, X_b_w)
  */
  inline CartesianVector cartesianErrorWorld(
    const CartesianPosition& Xd,
    const CartesianPosition& X)
  {
    CartesianVector e;
    e.linear() = Xd.linear() - X.linear();
    e.angular() = logErrorWorld(Xd.angular(), X.angular());
    return e;
  };

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired CartesianPosition Xd_b_w body wrt world
  * and the CartesianPosition X_b_w expressed in world.
  * 
  * @note The resulting 6d error expresses linear velocity in world frame
  * and angular velocity in body frame
  * 
  * \return cartesian error w.r.t world
  * e_b = cartesianError(Xd_b_w, X_b_w)
  */
  inline CartesianVector cartesianErrorBody(
    const CartesianPosition& Xd,
    const CartesianPosition& X)
  {
    CartesianVector e;
    e.linear() = Xd.linear() - X.linear();
    e.angular() = logErrorBody(Xd.angular(), X.angular());
    return e;
  }

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired CartesianPosition Td_b_w body wrt world
  * and the CartesianPosition T_b_w expressed in world.
  * 
  * @note The resulting 6d error expresses linear velocity in world frame
  * and angular velocity in world frame
  * 
  * \return cartesian error w.r.t world
  * e_w = cartesianError(Td_b_w, T_b_w)
  */
  inline CartesianVector cartesianErrorWorld(
    const HomogeneousTransformation& Td,
    const HomogeneousTransformation& T)
  {
    CartesianVector e;
    control_core::AngularPosition<Scalar> Qd(Td.orien());
    control_core::AngularPosition<Scalar> Q(T.orien());
    e.linear() = Td.pos() - T.pos();
    e.angular() = logErrorWorld(Qd, Q);
    return e;
  };

  /*!
  * \brief CartesianTransformation Error 
  * 
  * Computes the Error between a desired CartesianPosition Td_b_w body wrt world
  * and the CartesianPosition T_b_w expressed in world.
  * 
  * @note The resulting 6d error expresses linear velocity in world frame
  * and angular velocity in body frame
  * 
  * \return cartesian error w.r.t world
  * e_b = cartesianError(Td_b_w, T_b_w)
  */
  inline CartesianVector cartesianErrorBody(
    const HomogeneousTransformation& Td,
    const HomogeneousTransformation& T)
  {
    CartesianVector e;
    control_core::AngularPosition<Scalar> Qd(Td.orien());
    control_core::AngularPosition<Scalar> Q(T.orien());
    e.linear() = Td.pos() - T.pos();
    e.angular() = logErrorBody(Qd, Q);
    return e;
  }

  /**
   * @brief integrate body velocity cartesian position of body wrt world
   * 
   * @note The 6d velocity expresses linear velocity in world frame
   * and angular velocity in body frame
   * 
   * @param X_b_w CartesianPosition body wrt world
   * @param XP_b CarteisanVelocity expressed in body frame
   * @param dt 
   * @return CartesianPosition in the next step
   */
  inline CartesianPosition integrateVelocityBody(
    const CartesianPosition& X_b_w,
    const CartesianVelocity& XP_b, cc::Scalar dt)
  {
    CartesianPosition X;
    X.linear() = X_b_w.linear() + XP_b.linear()*dt;
    X.angular() = integrateVelocityBody(X_b_w.angular(), XP_b.angular(), dt);
    return X;
  }

  /**
   * @brief integrate body velocity expressed in body frame to 
   * cartesian position of body wrt world
   * 
   * @note The 6d velocity expresses linear velocity in world frame
   * and angular velocity in body frame
   * 
   * @param X_b_w CartesianPosition body wrt world
   * @param XP_b CarteisanVelocity expressed in world frame
   * @param dt 
   * @return CartesianPosition in the next step
   */
  inline CartesianPosition integrateVelocityWorld(
    const CartesianPosition& X_b_w,
    const CartesianVelocity& XP_w, cc::Scalar dt)
  {
    CartesianPosition X;
    X.linear() = X_b_w.linear() + XP_w.linear()*dt;
    X.angular() = integrateVelocityWorld(X_b_w.angular(), XP_w.angular(), dt);
    return X;
  }

  /**
   * @brief Integrate Acceleration and Velocity type expressed in body frame
   * into a position type describing body wrt world
   * 
   * @note The 6d velocity/acceleration expresses linear velocity in world frame
   * and angular velocity in body frame
   * 
   * @tparam _Derived 
   * @param state Current State 
   * @param next_state Next State
   * @param dt 
   */
  template <typename _Derived>
  inline void integrateStateBody(
    const control_core::StateBase<_Derived>& state,
    control_core::StateBase<_Derived>& next_state,
    cc::Scalar dt)
  {
    typedef _Derived Derived;
    typedef typename cc::traits<Derived>::Vel VP;

    VP v_mean = state.vel() + 0.5 * dt * state.acc();
    next_state.vel() = state.vel() + dt * state.acc();
    next_state.pos() = integrateVelocityBody(state.pos(), v_mean, dt);
  }

  /**
   * @brief Integrate Acceleration and Velocity type expressed in world frame
   * into a position type describing body wrt world
   * 
   * @note The 6d velocity/acceleration expresses linear velocity in world frame
   * and angular velocity in body frame
   * 
   * @tparam _Derived 
   * @param state Current State 
   * @param next_state Next State
   * @param dt 
   */
  template <typename _Derived>
  inline void integrateStateWorld(
    const control_core::StateBase<_Derived>& state,
    control_core::StateBase<_Derived>& next_state,
    cc::Scalar dt)
  {
    typedef _Derived Derived;
    typedef typename cc::traits<Derived>::Vel VP;

    VP v_mean = state.vel() + 0.5 * dt * state.acc();
    next_state.vel() = state.vel() + dt * state.acc();
    next_state.pos() = integrateVelocityWorld(state.pos(), v_mean, dt);
  }

  /**
   * @brief compute the average Cartesian Pose
   * 
   * Vector weights holds the weights of each Pose 
   * Note: sum(weights)=1
   * 
   * Vector Xs holds the Poses
   * 
   * @param weights 
   * @param Xs 
   * @return cc::CartesianPosition 
   */
  inline cc::CartesianPosition average(
    const std::vector<cc::Scalar>& weights, 
    const std::vector<cc::CartesianPosition>& Xs)
  {
    cc::CartesianPosition avg;
    avg.linear().setZero();
    Eigen::Matrix<cc::Scalar,4,-1> A(4,Xs.size());
    for(size_t i = 0; i < Xs.size(); ++i)
    {
      avg.linear() += weights[i]*Xs[i].linear();
      if(Xs[i].angular().coeffs()[3] < 0)
        A.col(i) = -weights[i]*Xs[i].angular().coeffs();
      else
        A.col(i) = weights[i]*Xs[i].angular().coeffs();
    }
    avg.angular() = average(A);
    return avg;
  }

  /**
   * @brief interpolate from pose_from to pose_to, parameterized by alpha
   * 
   * @param pose_from 
   * @param pose_to 
   * @param alpha 
   * @return cc::CartesianPosition 
   */
  inline cc::CartesianPosition interpolate(
    const CartesianPosition& pose_from,
    const CartesianPosition& pose_to,
    cc::Scalar alpha)
  {
    cc::CartesianPosition pose;
    
    alpha = std::max(0.0, std::min(1.0, alpha));
    pose.linear() = 
      pose_from.linear() + alpha*(pose_to.linear() - pose_from.linear());
    pose.angular() = pose_from.angular().slerp(alpha, pose_to.angular());
    return pose;
  }
}

#endif