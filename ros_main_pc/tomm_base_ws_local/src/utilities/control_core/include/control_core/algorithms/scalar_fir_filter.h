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


#ifndef CONTROL_CORE_SCALAR_FIR_H
#define CONTROL_CORE_SCALAR_FIR_H

#include <ros/assert.h>
#include <control_core/algorithms/i_scalar_filter.h>

#include <deque>

namespace control_core {

/*!
 * \brief The FIR Filter class.
 *
 * ScalarFIRFilter implements various Finite impulse responce filters
 *
 */
template <typename _Scalar>
class ScalarFIRFilter :
  public IScalarFilter<_Scalar>
{
public:
  typedef _Scalar Scalar;
  typedef IScalarFilter<_Scalar> IFilter;
  typedef typename Eigen::Matrix<_Scalar, Eigen::Dynamic, 1> Vector;
  typedef typename Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

enum WindowType
{
  BARLETT,  //!< BARLETT window
  HANNING,  //!< HANNING window
  HAMMING,  //!< HAMMING window
  BLACKMAN, //!< BLACKMAN window
  RECT,
};

public:
  static ScalarFIRFilter
    BarlettFRI(Scalar f_s, Scalar f_cut, Scalar order)
  {
    return ScalarFIRFilter(f_s, f_cut, order, BARLETT);
  }

  static ScalarFIRFilter
    HanningFRI(Scalar f_s, Scalar f_cut, Scalar order)
  {
    return ScalarFIRFilter(f_s, f_cut, order, HANNING);
  }

  static ScalarFIRFilter
    HammingFRI(Scalar f_s, Scalar f_cut, Scalar order)
  {
    return ScalarFIRFilter(f_s, f_cut, order, HAMMING);
  }

  static ScalarFIRFilter
    BlackmanFRI(Scalar f_s, Scalar f_cut, Scalar order)
  {
    return ScalarFIRFilter(f_s, f_cut, order, BLACKMAN);
  }

  static ScalarFIRFilter
    RectFRI(Scalar f_s, Scalar f_cut, Scalar order)
  {
    return ScalarFIRFilter(f_s, f_cut, order, RECT);
  }

protected:
  bool is_init_;                            //!< initalization flag
  bool is_valid_;                           //!< valid filter ouput flag
  int fill_cnt_;                            //!< counts buffer elements

  WindowType type_;                         //!< filter type
  int order_;                               //!< filter order
  int n_hist_;
  int mid_;

  Scalar fcut_;                             //!< cutoff frequencies
  Scalar f_s_;                              //!< sample frequency    

  std::vector<Scalar> w_sinc_;              //!< coeff
  std::vector<Scalar> w_win_;
  std::vector<Scalar> w_;

  std::deque<Scalar> s_;                    //!< filter buffer / state
  Scalar y_;                                //!< filter ouput

public:
  /*!
   * \brief Construct a FIR Lowpass Filter with sample
   * frequency, cutoff frequencies, filter order, windowing type.
   */
  ScalarFIRFilter(Scalar f_s,
            Scalar fcut,
            Scalar order,
            WindowType type = RECT) : 
    f_s_(f_s),
    fcut_(fcut),
    order_(order),
    type_(type),
    y_(0.0)
  {
    setup();
    reset();
  }

  /*!
   * \brief Copy constructor.
   * Copy Coeffecients, no need to recompute everything.
   */
  ScalarFIRFilter(const ScalarFIRFilter& other) :
    f_s_(other.f_s_),
    fcut_(other.fcut_),
    order_(other.order_),
    type_(other.type_),
    w_sinc_(other.w_sinc_),
    w_win_(other.w_win_),
    w_(other.w_),
    s_(other.s_),
    y_(0.0)
  {
    reset();
  }

  /*!
   * \brief Destructor.
   */
  virtual ~ScalarFIRFilter()
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
   * \brief Returns num of samples required for valid ouput
   * 
   * Return number of samples required for valid ouput.
   */
  int validCount() const
  {
    return order_;
  }

  /*!
   * \brief Create a copy of this object using the new operator.
   */
  IFilter* copy() const
  {
    return new ScalarFIRFilter(*this);
  }

  /*!
   * \brief Reset the algorithm to inital state.
   *
   * Ouput is initalized set in next call to update()
   */
  virtual Scalar reset()
  {
    is_init_ = false;
    is_valid_ = false;
    fill_cnt_ = 0;
    y_ = Scalar(0);
    return y_;
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
    fill_cnt_ = 0;
    y_ = x;
    return update(x); 
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
      initialize(x);
      is_init_ = true;
    }
    
    // check if next ouput is valid
    if(!is_valid_)
    {
      fill_cnt_++;
      if(fill_cnt_ >= order_)
      {
        is_valid_ = true;
      }
    }

    // apply the filter
    y_ = filter(x);
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
    return order_;
  }

  /*!
   * \brief the signal sampling frequency
   */
  Scalar sampleFrequency() const 
  {
    return f_s_;
  }

  /*!
   * \brief the filters cutoff frequency
   */
  Scalar lowerCutoffFrequency() const
  {
    return fcut_;
  }

  /*!
   * \brief the filters cutoff frequency
   */
  Scalar upperCutoffFrequency() const
  {
    return fcut_;
  }

  const std::vector<Scalar>& weights() const 
  {
    return w_;
  }

protected:
  /*!
    * \brief Setup the filter for given filter type.
    * 
    * Resize buffer arrays and compute filter coefficients.
    */
  void setup() 
  {
    // make sure that the order number is odd
    order_ = order_/2;
    order_ = 2 * order_ + 1;

    n_hist_ = order_ - 1;
    mid_ = n_hist_/2;

    Scalar ft = fcut_/f_s_;

    // sinc weights
    computeSinc(ft);

    // compute window weights
    w_win_.resize(order_, 1);
    switch (type_)
    {
      case BARLETT:
        setWindowBar();
        break;
      case HANNING:
        setWindowHann();
        break;
      case HAMMING:
        setWindowHamm();
        break;
      case BLACKMAN:
        setWindowBlack();
        break;
      default:
        break;
    }

    // compute normalized weights
    computeWeights();
  }

  /*!
    * \brief Initialize the filter state.
    * 
    * Fill buffer arrays such that ouput is x in the first few iterations
    */
  void initialize(Scalar x)
  {
    s_.resize(order_, x);
  }

  /*!
    * \brief Apply the filter algorithm.
    */
  Scalar filter(Scalar x)
  {
    s_.pop_front();
    s_.push_back(x);
    x = 0;
    for(size_t i=0; i < order_; ++i)
    {
      // convolution requires inverse transversation of one of the two arrays (ie.: y=sum(x(i)*w(N-i)) ), but since FIR is symmetric, this is not needed here
      x += s_[i] * w_[i];
    }
    return x;
  }

  /*!
    * \brief Setup computeSinc
    */
  void computeSinc(Scalar ft)
  {
    w_sinc_.resize(order_);
    for(size_t i = 0; i < order_; ++i)
    {
      if(i != n_hist_/2)
      {
        w_sinc_[i] = sin(2*M_PI*ft * (i - mid_)) / (M_PI * (i - mid_));
      }
      else
      {
        w_sinc_[i] = 2 * ft;
      }
    }
  }

  void setWindowBar()
  {
    for(int i=0; i < order_; i++)
    {
      w_win_[i] = 1 - 2*abs(i - mid_)/n_hist_;
    }
  }

  void setWindowHann()
  {
    for(int i=0; i < order_; i++)
    {
      w_win_[i] = 0.5 - 0.5 * cos(2*M_PI*i / n_hist_);
    }
  }

  void setWindowHamm()
  {
    for(int i=0; i < order_; i++)
    {
      w_win_[i]  = 0.54 - 0.46 * cos(2*M_PI*i / n_hist_);
    }
  }

  void setWindowBlack()
  {
    for(int i=0; i < order_; i++)
    {
      w_win_[i]  = 0.42 - 0.5 * cos(2*M_PI*i / n_hist_) + 0.08 * cos(4*M_PI*i / n_hist_);
    }
  }

  void computeWeights()
  {
    w_.resize(order_);
    // compute weights
    double w_sum = 0;
    for(int i=0; i < order_; i++)
    {
      w_[i] = w_sinc_[i] * w_win_[i];
      w_sum += w_[i];
    }
    // normalise them
    for(int i=0; i < order_; i++)
    {
      w_[i] /= w_sum;
    }
  }

}; 

} // namespace control_core

#endif
