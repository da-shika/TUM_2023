#ifndef ICS_TSID_COMMON_contact_6d_ext_hpp__
#define ICS_TSID_COMMON_contact_6d_ext_hpp__

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include "tsid/contacts/contact-6d.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/math/utils.hpp"

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/utilities/conversions.h>

namespace tsid
{
  namespace contacts
  {
    class Contact6dExt : public Contact6d
    {
    protected:
      unsigned int priority_level_; // priority level
      double motion_weight_;        // weight of motion task
      double weight_reg_;           // force regularization weight
      cc::Contact contact_;            // contact container

    public:
      Contact6dExt(
          const std::string &name,
          RobotWrapper &robot,
          const std::string &frameName,
          ConstRefMatrix contactPoints,
          ConstRefVector contactNormal,
          double frictionCoefficient,
          double minNormalForce,
          double maxNormalForce,
          double weight_reg,
          double motion_weight,
          unsigned int priority_level) : Contact6d(name, robot, frameName, contactPoints, contactNormal,
                                                   frictionCoefficient, minNormalForce, maxNormalForce),
                                         weight_reg_(weight_reg),
                                         motion_weight_(motion_weight),
                                         priority_level_(priority_level)
      {
        // setup contact
        contact_.setZero();
        contact_.hull().vertices() = contactPoints.topRows(2);
        contact_.friction() = frictionCoefficient;
      }

      virtual ~Contact6dExt() {};

      void setRegularizationWeight(double weight_reg)
      {
        weight_reg_ = weight_reg;
      }

      void setMotionWeight(double motion_weight)
      {
        motion_weight_ = motion_weight;
      }

      /**
       * @brief Set the Reference of motion task
       *
       * @param sample_ref
       */
      void setMotionReference(trajectories::TrajectorySample sample_ref)
      {
        m_motionTask.setReference(sample_ref);
      }

      /**
       * @brief Get the Reference of motion task
       *
       * @return const trajectories::TrajectorySample&
       */
      const trajectories::TrajectorySample &getMotionReference()
      {
        return m_motionTask.getReference();
      }

      /**
       * @brief get the position of motion task
       *
       * @return SE3
       */
      SE3 getMotionPosition() const
      {
        SE3 pose;
        auto vec = m_motionTask.position();
        math::vectorToSE3(vec, pose);
        return pose;
      }

      /**
       * @brief get the reference position of motion task
       *
       * @return SE3
       */
      SE3 getMotionReferencePosition() const
      {
        SE3 pose;
        auto vec = m_motionTask.getReference().getValue();
        math::vectorToSE3(vec, pose);
        return pose;
      }

      /**
       * @brief convert to Contact type
       *
       * @return cc::Contact
       */
      const cc::Contact& toContact()
      {
        contact_.pose() = ics::to_pose(getMotionPosition());
        return contact_;
      }
      
      /**
       * @brief convert to local PolygonShape
       *
       * @return cc::Contact
       */
      const cc::PolygonShape& getHull() const
      {
        return contact_.hull();
      }

      /**
       * @brief Get the Local Contact Points
       *
       * @return const Matrix3x&
       */
      const Matrix3x &getLocalContactPoints() const
      {
        return m_contactPoints;
      }

      /**
       * @brief Get the Global Contact Points
       *
       * note: This will only return valid results after the task has been
       * updated once. Be careful in the controllers start function!
       *
       * @return Matrix3x
       */
      Matrix3x getGlobalContactPoints() const
      {
        SE3 pose = getMotionPosition();
        return (pose.rotation() * m_contactPoints).colwise() + pose.translation();
      }

      /**
       * @brief Get the Global Contact Reference Points
       *
       * @return Matrix3x
       */
      Matrix3x getGlobalContactReferencePoints() const
      {
        SE3 pose = getMotionReferencePosition();
        return (pose.rotation() * m_contactPoints).colwise() + pose.translation();
      }

      /**
       * @brief Get the Center Point of the contact
       *
       * @return Vector3
       */
      Vector3 getCenterPoint() const
      {
        return getGlobalContactPoints().rowwise().mean();
      }

      /**
       * @brief Get the Reference Center Point of the contact
       *
       * @return Vector3
       */
      Vector3 getReferenceCenterPoint() const
      {
        return getGlobalContactReferencePoints().rowwise().mean();
      }

      const Vector6 getForceReference() const
      {
        return m_fRef;
      }

      double getRegularizationWeight() const
      {
        return weight_reg_;
      }

      double getMotionWeight() const
      {
        return motion_weight_;
      }

      unsigned int getPriorityLevel() const
      {
        return priority_level_;
      }
    };

  } // namespace contacts

} // namespace tsid

#endif