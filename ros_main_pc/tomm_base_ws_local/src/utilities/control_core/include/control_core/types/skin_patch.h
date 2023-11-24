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


#ifndef CONTROL_CORE_SKIN_PATCH_DATA_H
#define CONTROL_CORE_SKIN_PATCH_DATA_H

#include <control_core/types/skin_modality.h>
#include <control_core_msgs/SkinPatch.h>

namespace control_core{

/*!
 * \brief The SkinPatch class.
 *
 *  This class is a container for:
 *    - proximity
 *    - force
 *    - pose
 * 
 * Note: the proximity and force is expressed wrt to pose (default identity).
 * Pose is expressed wrt frame which is a joint/frame on the robot model.
 */
template <typename _Scalar>
class SkinPatch 
{
public:
  typedef _Scalar Scalar;
  typedef SkinModality<Scalar> Modality;

  /*!
   * \brief Construct as Zero.
   */
  static const SkinPatch& Zero()
  {
    static SkinPatch v;
    static bool once = false;
    if(!once)
    {
      v.frame().clear();
      v.pose().setZero();
      v.force().setZero();
      v.proximity().setZero();
      v.minDist() = 1.0;
      v.maxDist() = 1.0;
      once = true;
    }
    return v;
  }

protected:
  std::string frame_;             //!< reference frame
  cc::CartesianPosition pose_;    //!< reference pose wrt frame
  Modality force_;                //!< center of pressure
  Modality proximity_;            //!< wrench
  cc::Scalar min_dist_;           //!< minimum distance
  cc::Scalar max_dist_;           //!< maximum distance

public:

  /*!
    * \brief Default Constructor.
    */
  SkinPatch()
  {
  }

  /*!
   * \brief Copy constructor.
   */
  SkinPatch(const SkinPatch& other) :
    frame_(other.frame_),
    pose_(other.pose_),
    force_(other.force_),
    proximity_(other.proximity_),
    min_dist_(other.min_dist_),
    max_dist_(other.max_dist_)
  {
  }

  SkinPatch(const control_core_msgs::SkinPatch& other)
  {
    operator=(other);
  }

  void setZero()
  {
    pose().setZero();
    force().setZero();
    proximity().setZero();
    minDist() = 1.0;
    maxDist() = 1.0;
  }

  std::string& frame()
  {
    return frame_;
  }

  const std::string& frame() const
  {
    return frame_;
  }

  cc::CartesianPosition& pose()
  {
    return pose_;
  }

  const cc::CartesianPosition& pose() const
  {
    return pose_;
  }

  Modality& f()
  {
    return force_;
  }

  Modality& force()
  {
    return force_;
  }

  const Modality& f() const
  {
    return force_;
  }

  const Modality& force() const
  {
    return force_;
  }

  Modality& p()
  {
    return proximity_;
  }

  Modality& proximity()
  {
    return proximity_;
  }

  const Modality& p() const
  {
    return proximity_;
  }

  const Modality& proximity() const
  {
    return proximity_;
  }

  cc::Scalar& minDist()
  {
    return min_dist_; 
  }

  const cc::Scalar& minDist() const
  {
    return min_dist_; 
  }

  cc::Scalar& maxDist()
  {
    return max_dist_;
  }

  const cc::Scalar& maxDist() const
  {
    return max_dist_;
  }

  /*!
  * \brief Assignment of control_core_msgs::SkinPatch.
  */
  SkinPatch& operator=(const control_core_msgs::SkinPatch& msg)
  {
    frame() = msg.header.frame_id;
    pose() = msg.pose;
    force() = msg.force;
    proximity() = msg.proximity;
    maxDist() = msg.max_dist.data;
    minDist() = msg.min_dist.data;
    return *this;
  }

  /*!
  * \brief Conversion to control_core_msgs::SkinPatch.
  */
  operator control_core_msgs::SkinPatch() const
  {
    control_core_msgs::SkinPatch msg;
    msg.header.frame_id = frame();
    msg.pose = pose();
    msg.force = force();
    msg.proximity = proximity();
    msg.max_dist.data = maxDist();
    msg.min_dist.data = minDist();
    return msg;
  }  

  /*!
  * \brief Conversion to control_core_msgs::SkinPatch.
  */
  control_core_msgs::SkinPatch toSkinPatchMsg() const
  {
    return static_cast<control_core_msgs::SkinPatch>(*this);
  }

};

}

#endif // CONTROL_CORE_SKIN_PATCH_DATA_H
