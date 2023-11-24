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


#ifndef CONTROL_CORE_FT_SENSOR_H
#define CONTROL_CORE_FT_SENSOR_H

#include <control_core/types/wrench.h>
#include <geometry_msgs/WrenchStamped.h>

namespace control_core{

/*!
 * \brief The FtSensor class.
 *
 *  This class is a container for:
 *    - wrench measured by ft sensor
 *    - sensor frame on the robot
 */
template <typename _Scalar>
class FtSensor 
{
public:
  typedef _Scalar Scalar;
  typedef Wrench<Scalar> WrenchVec;

  /*!
   * \brief Construct as Zero.
   */
  static const FtSensor& Zero()
  {
    static FtSensor v;
    static bool once = false;
    if(!once)
    {
      v.wrench().setZero();
      once = true;
    }
    return v;
  }

protected:
  std::string frame_;       //!< robot frame
  WrenchVec wrench_;        //!< wrench vector

public:
  /*!
    * \brief Default Constructor.
    */
  FtSensor()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  FtSensor(const FtSensor& other) :
    frame_(other.frame_),
    wrench_(other.wrench_)
  {
  }

  void setZero()
  {
    wrench().setZero();
  }

  std::string& frame()
  {
    return frame_;
  }

  const std::string& frame() const
  {
    return frame_;
  }

  WrenchVec& W()
  {
    return wrench_;
  }

  WrenchVec& wrench()
  {
    return wrench_;
  }

  const WrenchVec& W() const
  {
    return wrench_;
  }

  const WrenchVec& wrench() const
  {
    return wrench_;
  }

  /*!
  * \brief Assignment of FtSensor.
  */
  FtSensor& operator=(const FtSensor& other)
  {
    wrench() = other.wrench();   
    frame() = other.frame();
    return *this;
  }

  /*!
  * \brief Assignment of geometry_msgs::WrenchStamped.
  */
  FtSensor& operator=(const geometry_msgs::WrenchStamped& msg)
  {
    wrench() = msg.wrench;
    frame() = msg.header.frame_id;
    return *this;
  }

  /*!
  * \brief Conversion to geometry_msgs::WrenchStamped.
  */
  operator geometry_msgs::WrenchStamped() const
  {
    geometry_msgs::WrenchStamped msg;
    msg.wrench() = wrench();
    msg.header.frame_id = frame();
    return msg;
  }  

  /*!
  * \brief Conversion to geometry_msgs::WrenchStamped.
  */
  geometry_msgs::WrenchStamped toWrenchStampedMsg() const
  {
    return static_cast<geometry_msgs::WrenchStamped>(*this);
  }

};

}

#endif // CONTROL_CORE_FT_SENSOR_H
