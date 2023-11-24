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

#ifndef CONTROL_CORE_ROBOT_STATE_H_
#define CONTROL_CORE_ROBOT_STATE_H_

#include <control_core/types/body_id.h>

#include <control_core/types/joint_state.h>
#include <control_core/types/cartesian_state.h>

#include <control_core/types/ft_sensor.h>
#include <control_core/types/imu_sensor.h>
#include <control_core/types/skin_patch.h>

#include <control_core_msgs/RobotState.h>

namespace control_core 
{

  template<typename _Scalar, int _Rows = Eigen::Dynamic>
  class RobotState
  {
  public:
    enum
    {
      RowsAtCompileTime = _Rows,
    };
    typedef _Scalar Scalar;
    typedef JointState<Scalar> JState;
    typedef CartesianState<Scalar> CState;
    typedef FtSensor<Scalar> ForceTorqueSensor;
    typedef ImuSensor<Scalar> FloatingBaseImu;
    typedef SkinPatch<Scalar> Patch;

    /*!
    * \brief Construct as Zero.
    */
    static const RobotState& Zero()
    {
      static RobotState v;
      static bool once = false;
      if(!once)
      {
        v.joints().setZero();
        v.floatingBase().setZero();
        v.imu().setZero();
        v.ft_sensor_.resize(4, ForceTorqueSensor::Zero());
        v.body_states_.resize(4, CState::Zero());
        v.patches_.resize(4, Patch::Zero());
        once = true;
      }
      return v;
    }

    /*!
    * \brief Get Zero State for dynamic Sizes
    */
   template<typename T>
    static const RobotState& Zero(const T& x)
    {
      static RobotState v;
      v.joints().setZero(x);
      v.floatingBase().setZero();
      v.imu().setZero();
      v.ft_sensor_.resize(4, ForceTorqueSensor::Zero());
      v.body_states_.resize(4, CState::Zero());
      v.patches_.resize(4, Patch::Zero());
      return v;
    }

  private:
    JState joints_;
    CState floating_base_;

    std::vector<CState> body_states_;
    std::vector<ForceTorqueSensor> ft_sensor_;
    std::vector<Patch> patches_;

    FloatingBaseImu imu_;

  public:
    RobotState()
    {
      ft_sensor_.resize(4);
      patches_.resize(4);
      body_states_.resize(4);
    }

    RobotState(const RobotState& other) : 
      joints_(other.joints_),
      floating_base_(other.floating_base_),
      imu_(other.imu_),
      ft_sensor_(other.ft_sensor_),
      body_states_(other.body_states_),
      patches_(other.patches_)
    {
    }

    JState& joints()
    {
      return joints_;
    }

    const JState& joints() const
    {
      return joints_;
    }

    CState& floatingBase()
    {
      return floating_base_;
    }

    const CState& floatingBase() const
    {
      return floating_base_;
    }

    const ForceTorqueSensor& ftSensor(BodyId body_id) const
    {
      return ft_sensor_[body_id];
    }

    ForceTorqueSensor& ftSensor(BodyId body_id)
    {
      return ft_sensor_[body_id];
    }

    const std::vector<ForceTorqueSensor>& ftSensors() const
    {
      return ft_sensor_;
    }

    std::vector<ForceTorqueSensor>& ftSensors()
    {
      return ft_sensor_;
    }

    const CState& body(BodyId body_id) const
    {
      return body_states_[body_id];
    }

    CState& body(BodyId body_id)
    {
      return body_states_[body_id];
    }

    const std::vector<CState>& bodies() const
    {
      return body_states_;
    }

    std::vector<CState>& bodies()
    {
      return body_states_;
    }

    const FloatingBaseImu& imu() const
    {
      return imu_;
    }

    FloatingBaseImu& imu()
    {
      return imu_;
    }

    Patch& patch(BodyId body_id)
    {
      return patches_[body_id];
    }

    const Patch& patch(BodyId body_id) const
    {
      return patches_[body_id];
    }

    RobotState& operator=(const RobotState& other)
    {
      joints() = other.joints_;
      floatingBase() = other.floating_base_;
      imu() = other.imu_;
      ft_sensor_ = other.ft_sensor_;
      patches_ = other.patches_;
      return *this;
    }

    /*!
    * \brief Assignment of control_core_msgs::RobotState.
    */
    RobotState& operator=(const control_core_msgs::RobotState& msg)
    {
      joints() = msg.joints;
      floatingBase() = msg.floating_base;
      imu() = msg.imu;
      for(size_t i = 0; i < std::min(msg.ft_sensors.size(),size_t(4)); ++i)
        ft_sensor_[i] = msg.ft_sensors[i];
      for(size_t i = 0; i < std::min(msg.patches.size(),size_t(4)); ++i)
        patches_[i] = msg.patches[i];
      return *this;
    }

    /*!
    * \brief Conversion to control_core_msgs::CartesianState.
    */
    operator control_core_msgs::RobotState() const
    {
      control_core_msgs::RobotState msg;
      msg.joints = joints();
      msg.floating_base = floatingBase();
      msg.ft_sensors.resize(4);
      for(size_t i = 0; i < 4; ++i)
        msg.ft_sensors[i] = ft_sensor_[i];
      msg.patches.resize(4);
      for(size_t i = 0; i < 4; ++i)
        msg.patches[i] = patches_[i];
      return msg;
    }  

    /*!
    * \brief Conversion to control_core_msgs::CartesianState.
    */
    control_core_msgs::RobotState toRobotStateMsg() const
    {
      return static_cast<control_core_msgs::RobotState>(*this);
    }

    /**
     * @brief set zero for static type
     * 
     */
    void setZero()
    {
      joints().setZero();
      floatingBase().setZero();
      imu().setZero();
      ft_sensor_.resize(4, ForceTorqueSensor::Zero());
      patches_.resize(4, Patch::Zero());
    }

    /**
     * @brief set zero for dynamic type
     * 
     * @param x 
     */
    void setZero(int x)
    {
      joints().setZero(x);
      floatingBase().setZero();
      imu().setZero();  
      ft_sensor_.resize(4, ForceTorqueSensor::Zero());
      patches_.resize(4, Patch::Zero());
    }

    /**
     * @brief to string
     *
     */
    std::string toString() const
    {
      std::stringstream ss;
      ss << "joints.pos=" << joints().pos().toString() << std::endl;
      ss << "joints.vel=" << joints().vel().toString() << std::endl;
      ss << "floatingBase=" << floatingBase().toString() << std::endl;
      for(size_t i = 0; i < 4; ++i) {
        ss << "ft_sensor[" << i << "].frame=" << ft_sensor_[i].frame() << std::endl;
        ss << "ft_sensor[" << i << "].wrench=" << ft_sensor_[i].wrench().toString() << std::endl;
      }
      ss << "imu.frame=" << imu().frame() << std::endl;
      ss << "imu.orientation=" << imu().angularPos().toString() << std::endl;
      ss << "imu.omega=" << imu().omega().toString() << std::endl;
      ss << "imu.linear_acc=" << imu().linearAcc().toString() << std::endl;
      return ss.str();
    }

  };

}

#endif