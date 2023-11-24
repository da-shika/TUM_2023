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

#ifndef CONTROL_CORE_SKIN_MODALITY_H
#define CONTROL_CORE_SKIN_MODALITY_H

#include <control_core/types/linear_position.h>
#include <control_core/types/wrench.h>
#include <control_core/geometry/shapes.h>

#include <control_core_msgs/SkinModality.h>

namespace control_core{

/*!
 * \brief The SkinModality class.
 *
 *  This class is a container for:
 *    - center of pressure
 *    - Wrench
 *    - 2d ConevexHull of the actived cells 
 */
template <typename _Scalar>
class SkinModality 
{
public:
  typedef _Scalar Scalar;
  typedef LinearPosition<Scalar> LinearPos;
  typedef Wrench<Scalar> SkinWrench;
  typedef cc::PolygonShape Hull;

  /*!
   * \brief Construct as Zero.
   */
  static const SkinModality& Zero()
  {
    static SkinModality v;
    static bool once = false;
    if(!once)
    {
      v.setZero();
      once = true;
    }
    return v;
  }

protected:
  cc::Scalar area_;       //!< number active cells
  LinearPos cop_;         //!< center of pressure
  SkinWrench wrench_;     //!< wrench
  Hull hull_;             //!< convex hull
  double min_;            //!< min measurment
  double max_;            //!< max measurment

public:
  /*!
    * \brief Default Constructor.
    */
  SkinModality()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  SkinModality(const SkinModality& other) :
    area_(other.area_),
    cop_(other.cop_),
    wrench_(other.wrench_),
    hull_(other.hull_)
  {
  }

  void setZero()
  {
    min() = 1.0;
    max() = 1.0;
    area() = 0.0;
    cop().setZero();
    wrench().setZero();
  }

  cc::Scalar& area()
  {
    return area_;
  }

  const cc::Scalar& area() const
  {
    return area_;
  }

  cc::Scalar& min()
  {
    return min_;
  }

  const cc::Scalar& min() const
  {
    return min_;
  }

  cc::Scalar& max()
  {
    return max_;
  }

  const cc::Scalar& max() const
  {
    return max_;
  }

  LinearPos& cop()
  {
    return cop_;
  }

  LinearPos& centerOfPressure()
  {
    return cop_;
  }

  const LinearPos& cop() const
  {
    return cop_;
  }

  const LinearPos& centerOfPressure() const
  {
    return cop_;
  }

  SkinWrench& W()
  {
    return wrench_;
  }

  SkinWrench& wrench()
  {
    return wrench_;
  }

  const SkinWrench& W() const
  {
    return wrench_;
  }

  const SkinWrench& wrench() const
  {
    return wrench_;
  }

  Hull& hull()
  {
    return hull_;
  }

  const Hull& hull() const
  {
    return hull_;
  }

  /*!
  * \brief Assignment of control_core_msgs::SkinModality.
  */
  SkinModality& operator=(const control_core_msgs::SkinModality& msg)
  {
    min() = msg.min.data;
    max() = msg.max.data;
    area() = msg.area.data;
    cop() = msg.cop;
    wrench() = msg.wrench;
    hull() = msg.hull;
    return *this;
  }

  /*!
  * \brief Conversion to control_core_msgs::SkinModality.
  */
  operator control_core_msgs::SkinModality() const
  {
    control_core_msgs::SkinModality msg;
    msg.min.data = min();
    msg.max.data = max();
    msg.area.data = area();
    msg.cop = cop();
    msg.wrench = wrench();
    msg.hull = hull();
    return msg;
  }  

  /*!
  * \brief Conversion to control_core_msgs::SkinModality.
  */
  control_core_msgs::SkinModality toSkinModalityMsg() const
  {
    return static_cast<control_core_msgs::SkinModality>(*this);
  }

};

}

#endif // CONTROL_CORE_SKIN_DATA_H
