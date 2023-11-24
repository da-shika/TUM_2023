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


#ifndef CONTROL_CORE_IMU_SENSOR_H
#define CONTROL_CORE_IMU_SENSOR_H

#include <control_core/types/linear_acceleration.h>
#include <control_core/types/angular_velocity.h>
#include <control_core/types/angular_position.h>

#include <sensor_msgs/Imu.h>

namespace control_core{

/*!
 * \brief The ImuSensor class.
 *
 *  This class is a container for:
 *    - linear acceleration xPP measured by the acceleration sensor
 *    - Angular velocity omega measured by the gyro sensor
 *    - AngularPosition Q computed by from the two
 */
template <typename _Scalar>
class ImuSensor 
{
public:
  typedef _Scalar Scalar;
  typedef LinearAcceleration<Scalar> LinearAcc;
  typedef AngularVelocity<Scalar> AngularVel;
  typedef AngularPosition<Scalar> AngularPos;

  /*!
   * \brief Construct as Zero.
   */
  static const ImuSensor& Zero()
  {
    static ImuSensor v;
    static bool once = false;
    if(!once)
    {
      v.angularPos().setZero();
      v.angularVel().setZero();
      v.linearAcc().setZero();  
      once = true;
    }
    return v;
  }

protected:
  std::string frame_;       //!< robot frame
  LinearAcc xPP_;           //!< accelerometer ouput
  AngularVel omega_;        //!< gyroscope ouput    
  AngularPos Q_;            //!< orientation wrt to robot base

public:
  /*!
    * \brief Default Constructor.
    */
  ImuSensor()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  ImuSensor(const ImuSensor& other) :
    frame_(other.frame_),
    xPP_(other.xPP_),
    omega_(other.omega_),
    Q_(other.Q_)
  {
  }

  void setZero()
  {
    angularPos().setZero();
    angularVel().setZero();
    linearAcc().setZero();  
  }

  std::string& frame()
  {
    return frame_;
  }

  const std::string& frame() const
  {
    return frame_;
  }

  LinearAcc& xPP()
  {
    return xPP_;
  }

  LinearAcc& linearAcc()
  {
    return xPP_;
  }

  const LinearAcc& xPP() const
  {
    return xPP_;
  }

  const LinearAcc& linearAcc() const
  {
    return xPP_;
  }

  AngularVel& omega()
  {
    return omega_;
  }

  AngularVel& angularVel()
  {
    return omega_;
  }

  const AngularVel& omega() const
  {
    return omega_;
  }

  const AngularVel& angularVel() const
  {
    return omega_;
  }

  AngularPos& Q()
  {
    return Q_;
  }

  AngularPos& angularPos()
  {
    return Q_;
  }

  const AngularPos& Q() const
  {
    return Q_;
  }

  const AngularPos& angularPos() const
  {
    return Q_;
  }

  /*!
  * \brief Assignment of ImuSensor.
  */
  ImuSensor& operator=(const ImuSensor& other)
  {
    angularPos() = other.angularPos();
    angularVel() = other.angularVel();
    linearAcc() = other.linearAcc();    
    frame() = other.frame();
    return *this;
  }

  /*!
  * \brief Assignment of sensor_msgs::Imu.
  */
  ImuSensor& operator=(const sensor_msgs::Imu& msg)
  {
    angularPos() = msg.orientation;
    angularVel() = msg.angular_velocity;
    linearAcc() = msg.linear_acceleration;    
    frame() = msg.header.frame_id;
    return *this;
  }

  /*!
  * \brief Conversion to sensor_msgs::Imu.
  */
  operator sensor_msgs::Imu() const
  {
    sensor_msgs::Imu msg;
    msg.orientation = angularPos();
    msg.angular_velocity = angularVel();
    msg.linear_acceleration = linearAcc();
    msg.header.frame_id = frame();
    return msg;
  }  

  /*!
  * \brief Conversion to sensor_msgs::Imu.
  */
  sensor_msgs::Imu toImuMsg() const
  {
    return static_cast<sensor_msgs::Imu>(*this);
  }

};

}

#endif // CONTROL_CORE_IMU_SENSOR_H
