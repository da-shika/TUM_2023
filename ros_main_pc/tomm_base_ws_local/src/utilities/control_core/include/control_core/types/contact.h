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


#ifndef CONTROL_CORE_CONTACT_H
#define CONTROL_CORE_CONTACT_H

#include <control_core/types/cartesian_position.h>
#include <control_core/types/vector3.h>
#include <control_core/types/wrench.h>
#include <control_core/geometry/shapes.h>

#include <control_core/math/adjoint.h>
#include <control_core/math/contacts.h>

#include <control_core_msgs/Contact.h>

namespace control_core 
{

  /**
   * @brief Contact Class
   * 
   * Stores the geometric and force information of a contact.
   * 
   * geometric:
   * - pose of contact frame in world
   * - sequence of point that formes contact hull (local)
   * - optional: offset on geometric center (e.g. for zmp target)
   * 
   * force:
   * - contact wrench in contact frame (local)
   * 
   * friction cones (computed on demand, otherwise empty)
   * - Grasp Matrix G
   * - H-rep Matrix H
   * - V-rep Matrix V
   * 
   * @tparam _Scalar 
   */
  template<typename _Scalar>
  class Contact
  {
  public:
    typedef _Scalar Scalar;
    typedef CartesianPosition<Scalar> Pose;
    typedef LinearPosition<Scalar> Pos;
    typedef Wrench<Scalar> W;
    typedef cc::PolygonShape Hull;
    typedef Eigen::Matrix<Scalar,3,Eigen::Dynamic> Vertices;

    typedef Eigen::Matrix<Scalar,-1,-1> MatrixX;
    typedef Eigen::Matrix<Scalar,-1,1> VectorX;

    typedef Eigen::Matrix<Scalar,16,6> RectangularWrenchIEq;

    /*!
    * \brief Construct as Zero.
    */
    static const Contact& Zero()
    {
      static Contact v;
      static bool once = false;
      if(!once)
      {
        v.pose().setZero();
        v.offset().setZero();
        v.friction() = 0.5;
        v.wrench().setZero();
        once = true;
      }
      return v;
    }

  private:
    // geometry
    Pose pose_;         // pose wrt world    
    Scalar friction_;   // friction coefficient
    Hull hull_;         // hull expressed wrt pose
    Pos offset_;        // aditional offset on the zmp target

    // force
    W wrench_;          // contact wrench

    // only if explicitly computed
    MatrixX G_;         // contact grasp matrix
    MatrixX H_;         // contact H-rep (inequalities)
    MatrixX V_;         // contact V-rep (vertices)

  public:
    Contact()
    {
    }

    Contact(const Contact& other) : 
      pose_(other.pose_),
      hull_(other.hull_),
      offset_(other.offset_),
      friction_(other.friction_),
      wrench_(other.wrench_)
    {
    }

    Contact(
      const Pose& pose, const Hull& hull, 
      const Pos& offset=Pos::Zero(), Scalar friction=0.5,
      const W& wrench=W::Zero()) :
        pose_(pose),
        hull_(hull),
        offset_(offset),
        friction_(friction),
        wrench_(wrench)
    {
    }

    Contact(
        const Pose& pose, Scalar x_half, Scalar y_half, 
        const Pos& offset = Pos::Zero(), Scalar friction=0.5,
        const W& wrench=W::Zero()) :
      pose_(pose),
      hull_(cc::PolygonShape(x_half, y_half)),
      offset_(offset),
      friction_(friction),
      wrench_(wrench)
    {
    }

    Contact(
        const Pose& pose, Scalar x_min, Scalar x_max, Scalar y_min, Scalar y_max, 
        const Pos& offset=Pos::Zero(), Scalar friction=0.5,
        const W& wrench=W::Zero()) :
      pose_(pose),
      hull_(cc::PolygonShape(x_min, x_max, y_min, y_max)),
      offset_(offset),
      friction_(friction),
      wrench_(wrench)
    {
    }

    /**
     * @brief foot pose of the foot
     * 
     * @return Pose& 
     */
    Pose& pose()
    {
      return pose_;
    }

    /**
     * @brief foot pose of the foot wrt world
     * 
     * @return Pose& 
     */
    const Pose& pose() const
    {
      return pose_;
    }

    /**
     * @brief offset translation target wrt foot
     * 
     * @return const Pos& 
     */
    const Pos& offset() const
    {
      return offset_;
    }

    /**
     * @brief offset translation target wrt foot
     * 
     * @return const Pos& 
     */
    Pos& offset()
    {
      return offset_;
    }

    /**
     * @brief target pose for the zmp wrt world, 
     * 
     * note this is always set to be the the center of the hull + offset
     * 
     * @return Pose 
     */
    Pose target() const
    {
      cc::Vector3 local = offset_;
      local.head(2) += hull_.centroid();

      Pose target;
      target.angular() = pose_.angular();
      target.linear() = 
        pose_.linear() + pose_.angular().toRotationMatrix()*local;
      return target;
    }

    /**
     * @brief target pose for the zmp wrt foot pose
     * 
     * note this is always set to be the the center of the hull + offset
     * 
     * @return Pose 
     */
    Pose targetLocal() const
    {
      cc::Vector3 local = offset_;
      local.head(2) += hull_.centroid();
      return Pose(cc::Rotation3::Identity(), local);
    }

    const Hull& hull() const
    {
      return hull_;
    }

    Hull& hull()
    {
      return hull_;
    }

    Scalar friction() const
    {
      return friction_;
    }

    Scalar& friction()
    {
      return friction_;
    }

    const W& wrench() const
    {
      return wrench_;
    }

    W& wrench()
    {
      return wrench_;
    }

    Contact& operator=(const Contact& other)
    {
      pose_ = other.pose_;
      hull_ = other.hull_;
      offset_ = other.offset_;
      wrench_ = other.wrench_;
      friction_ = other.friction_;
      G_ = other.G_;
      H_ = other.H_;
      V_ = other.V_;
      
      return *this;
    }

    /*!
    * \brief Assignment of control_core_msgs::Contact.
    */
    Contact& operator=(const control_core_msgs::Contact& msg)
    {
      pose_ = msg.pose;
      hull_ = msg.hull;
      offset_ = msg.offset;
      friction_ = msg.friction.data;
      return *this;
    }

    /*!
    * \brief Conversion to control_core_msgs::Contact.
    */
    operator control_core_msgs::Contact() const
    {
      control_core_msgs::Contact msg;
      msg.pose = pose_;
      msg.hull = hull_;
      msg.offset = offset_;
      msg.friction.data = friction_;
      return msg;
    }  

    /**
     * @brief set zero
     * 
     */
    void setZero()
    {
      pose_.setZero();
      offset_.setZero();
      wrench_.setZero();
    }

    Pos sagittal() const 
    {
      return pose_.angular().toRotationMatrix().col(0);
    }

    Pos lateral() const
    {
      return pose_.angular().toRotationMatrix().col(1);
    }

    Pos normal() const
    {
      return pose_.angular().toRotationMatrix().col(2);
    }

    Pos maxVertexWorld() const 
    {
      return verticesGlobal().rowwise().maxCoeff();
    }

    Pos minVertexWorld() const
    {
      return verticesGlobal().rowwise().minCoeff();
    }

    /**
     * @brief local hull vertices 
     * 
     * @return Vertices 
     */
    Vertices verticesLocal() const
    {
      Vertices vertices(3, hull_.vertices().cols());
      vertices.topRows(2) = hull_.vertices();
      vertices.bottomRows(1).setZero();
      return vertices;
    }

    /**
     * @brief global hull vertices
     * 
     * @return Vertices 
     */
    Vertices verticesGlobal() const
    {
      return (pose_.angular().toRotationMatrix()*verticesLocal()).colwise() + pose_.linear();
    }

    /**
     * @brief global hull in H-Rep
     * 
     */
    Hull::HalfSpace hullHalfSpaceGlobal() const
    {
      Hull::HalfSpace half_space = hull_.halfSpace();
      half_space.first = half_space.first*pose_.angular().toRotationMatrix2d().transpose();
      half_space.second += half_space.first*pose_.linear().head(2);
      return half_space;
    }

    /**
     * @brief returns the V-representation in the local frame (wrench cone)
     * 
     * @return const MatrixX& 
     */
    const MatrixX& rays() const
    {
      return V_;
    }

    MatrixX& rays()
    {
      return V_;
    }

    /**
     * @brief returns the H-representation in the local frame (wrench cone)
     * 
     * @return const MatrixX& 
     */
    const MatrixX& halfSpace() const
    {
      return H_;
    }

    MatrixX& halfSpace()
    {
      return H_;
    }

    /**
     * @brief returns the grasp matrix in the local frame
     * 
     * @return const MatrixX& 
     */
    const MatrixX& graspMatrix() const
    {
      return G_;
    }

    MatrixX& graspMatrix()
    {
      return G_;
    }

    std::string toString() const
    {
      std::stringstream ss;
      ss << "pose=    " << pose().toString() << std::endl;
      ss << "offset_= " << offset().toString() << std::endl;
      ss << "target=  " << target().toString() << std::endl;
      ss << "hull=\n  " << hull_.vertices() << std::endl;
      return ss.str();
    }

  };

}

#endif