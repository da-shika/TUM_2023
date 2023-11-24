/*! \file
 *
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author J. Rogelio Guadarrama-Olvera
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 20.02.2020
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
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#ifndef CONTROL_CORE_SCALAR_LEAKY_INTEGRATOR_H
#define CONTROL_CORE_SCALAR_LEAKY_INTEGRATOR_H

#include <Eigen/Dense>
#include <control_core/algorithms/i_scalar_filter.h>

namespace control_core{
/*!
 * \brief The ScalarLeakyIntegrator class.
 *
 * ScalarLeakyIntegrator is an Integrator with build in 
 * anti-windup strategy
 * 
 * ydot = x - alpha * y
 * 
 * See: https://en.wikipedia.org/wiki/Leaky_integrator
 */
template <typename _Scalar>
class ScalarLeakyIntegrator : public IScalarFilter<_Scalar>
{
public:
  typedef _Scalar Scalar;
  typedef IScalarFilter<_Scalar> IFilter;

protected:
  bool is_init_;              //!< initalization flag
  bool is_valid_;             //!< valid filter ouput flag
  Scalar alpha_;              //!< decay factors
  Scalar dt_;
  Scalar y_;                   //!< ouput

public:
  static ScalarLeakyIntegrator ExpDecay(Scalar alpha)
  {
    static ScalarLeakyIntegrator filter(alpha);
    return filter;
  }

public:
  /*!
  * \brief Constructor.
  * 
  * \param tau
  *   Decay time to 0.63*x_init
  *    
  * \param t_sample
  *   Sample time of signal   
  */
  ScalarLeakyIntegrator(Scalar tau, Scalar t_sample) :
    alpha_(std::exp(-t_sample/tau)),
    dt_(t_sample)
  {
  }

  /*!
   * \brief Copy constructor.
   */
  ScalarLeakyIntegrator(const ScalarLeakyIntegrator& other) :
    alpha_(other.alpha_)
  {
  }

  /*!
   * \brief Destructor.
   */
  virtual ~ScalarLeakyIntegrator()
  {
  }

  /*!
   * \brief Returns whether the ouput is valid
   * 
   * Return true if algorithm accumulated enouth samples to fill buffer.
   */
  bool valid() const
  {
    return is_valid_;
  }

  /*!
   * \brief Create a copy of this object using the new operator.
   */
  IFilter* copy() const
  {
    return new ScalarLeakyIntegrator(*this);
  }

 /*!
   * \brief Set Sample Frequency
   *
   * Sample frequency is a paramer that applies to all time based algorithms
   * (Special case we consider here)
   * 
   * Not Applicable
   */
  void setSampleFrequency(Scalar f_s)
  {
    // do nothing
  }

  void setAlpha(Scalar alpha)
  {
    alpha_ = alpha;
  }

  /*!
   * \brief Reset the algorithm to inital state \f$\mathScalarQ{x}{}\f$.
   *
   * Algorithm uses \f$\mathScalarQ{x}{}\f$ as inital value.
   */
  Scalar reset(Scalar x)
  {
    is_init_ = false;
    is_valid_ = false;
    return update(x);
  }

  /*!
   * \brief Reset the algorithm to inital state.
   *
   * Ouput is initalized set in next call to update()
   */
  Scalar reset()
  {
    is_init_ = false;
    is_valid_ = false;
    y_ = 0;
    return y_;
  }

  /*!
   * \brief Adds new sample \f$\mathScalarQ{x}{}\f$ wihtout computation.
   * 
   * Could be used to populate buffer, before algorithm is started.
   */
  void add(Scalar x)
  {
    // initalize buffer if is not initalized
    if(!is_init_) 
    {
      y_ = x;
      is_init_ = true;
      is_valid_ = true;
    }

    // in this algorithm: seperating add() and update() makes no sense,
    // since new value changes buffer state completely. So do it here.
    y_ = (1 - alpha_*dt_)*y_ + x*dt_;
  }

  /*!
   * \brief Update the filter for the next step.
   */
  Scalar update(Scalar x)
  {
    // add sample
    add(x);
    return y_;
  }

  /*!
   * \brief The current value of the signal.
   */
  Scalar output() const
  {
    return y_;
  }

  /*!
   * \brief the filters order
   */
  int order() const
  {
    return Scalar(0);
  }

  /*!
   * \brief the signal sampling frequency
   */
  Scalar sampleFrequency() const 
  {
    return Scalar(1);
  }

  /*!
   * \brief the filters cutoff frequency
   */
  Scalar lowerCutoffFrequency() const
  {
    return Scalar(0);
  }

  /*!
   * \brief the filters cutoff frequency
   */
  Scalar upperCutoffFrequency() const
  {
    return Scalar(0);
  }
};

} // namespace control_core

#endif