//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_task_joint_posture_mobile_hpp__
#define __invdyn_task_joint_posture_mobile_hpp__

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/tasks/task-motion.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/math/constraint-equality.hpp>

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/interfaces/tsid_wrapper_interface.h>

////////////////////////////////////////////////////////////////////////////////
// ros includes
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>

namespace tsid
{
  namespace tasks
  {

    class TaskJointPostureMobile : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::VectorXi VectorXi;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;

      static std::shared_ptr<TaskBase> Load(
        ics::TSIDWrapperInterface& tsid_wrapper,
        ros::NodeHandle& nh,
        const std::string& name,
        bool activate = true,
        bool verbose = true);

    public:
      TaskJointPostureMobile(const std::string & name,
                      RobotWrapper & robot);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(const TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      const Vector & getDesiredAcceleration() const;
      Vector getAcceleration(ConstRefVector dv) const;

      virtual void setMask(math::ConstRefVector mask);

      const Vector & position_error() const;
      const Vector & velocity_error() const;
      const Vector & position() const;
      const Vector & velocity() const;
      const Vector & position_ref() const;
      const Vector & velocity_ref() const;

      const Vector & Kp();
      const Vector & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

    protected:
      Vector m_Kp;
      Vector m_Kd;
      Vector m_p_error, m_v_error;
      Vector m_p, m_v;
      Vector m_a_des;
      VectorXi m_activeAxes;
      TrajectorySample m_ref;
      ConstraintEquality m_constraint;
    };
    
  }
}

#endif
