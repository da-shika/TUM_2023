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


#ifndef CONTROL_CORE_QUATERNION_FILTER_H
#define CONTROL_CORE_QUATERNION_FILTER_H

#include <control_core/algorithms/scalar_fir_filter.h>
#include <control_core/types/angular_position.h>
#include <deque>

#include <control_core/math/quaternion.h>

namespace control_core{

/**
 * @brief Quaternion Filter class
 * 
 * Filter quaternion based on Scalar FIR filter.
 * Computes a weighted average over quaternion buffer using FIR filter 
 * coefficients.
 * 
 */
template <typename _Scalar>
class QuaternionFilter
{
  public:
    typedef _Scalar Scalar;
    typedef ScalarFIRFilter<Scalar> Filter;
    typedef control_core::AngularPosition<Scalar> Quaternion;

  protected:
    bool is_init_;                        //!< is initalized
    Eigen::Matrix<Scalar,4,-1> S_;        //!< state matrix
    Eigen::Matrix<Scalar,4,-1> Sw_;       //!< weighted state matrix
    Filter filter_;                       //!< scalar filter
    Quaternion y_;                        //!< output

  public:
    QuaternionFilter(const Filter& filter) : 
      filter_(filter),
      is_init_(false)
    {
    }
    
    /*!
    * \brief Deconstructor.
    */
    virtual ~QuaternionFilter()
    {
    }

    /*!
    * \brief Returns whether the ouput is valid
    * 
    * Return true if algorithm accumulated enouth samples to fill buffer.
    */
    bool valid() const 
    {
      return is_init_;
    }

    virtual const Quaternion& value() const
    {
      return y_;
    }

    /*!
    * \brief Reset the algorithm to inital state.
    *
    * Use the current settings and start from scratch.
    * Sets input as inital value.
    */
    virtual const Quaternion& reset(const Quaternion& x)
    {
      is_init_ = false;
      y_ = x;
      return update(x);
    }

    /*!
    * \brief Reset the algorithm to inital state.
    *
    * Use the current settings and start from scratch.
    */
    virtual const Quaternion& reset()
    {
      return reset(Quaternion::Identity());
    }

    /*!
    * \brief Adds new sample to algorithm
    *
    * Could be used to populate buffer, before algorithm is used by update.
    */
    virtual void add(const Quaternion& x)
    {
      // initalize buffer if is not initalized
      if(!is_init_) 
      {
        initialize(x);
        is_init_ = true;
      }
      y_ = filter(x);
    }

    /*!
    * \brief Update the filter for the next step.
    * 
    */
    virtual const Quaternion& update(const Quaternion& x) 
    {
      // add sample
      add(x);
      return y_;
    }
  
  protected:
    virtual void initialize(const Quaternion& x)
    {
      S_.resize(4, filter_.order());
      for(size_t i = 0; i < S_.cols(); ++i)
        S_.col(i) = x.coeffs();
      Sw_ = S_;
    }

    virtual Quaternion filter(const Quaternion& x)
    {
      // shift state to the left
      Eigen::MatrixXd tmp = S_.rightCols(filter_.order()-1);
      S_.leftCols(filter_.order()-1) = tmp;
      S_.col(filter_.order()-1) = x.coeffs();

      // compute weighted average
      const std::vector<double>& w = filter_.weights();
      for(size_t i=0; i < filter_.order(); ++i)
        Sw_.col(i) = S_.col(i) * w[i];
      y_ = cc::average(Sw_);
      cc::checkFlipQuaternionSign(y_);

      return y_;
    }
};

}

#endif