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

#include <ics_tsid_tasks/tsid/task_joint_posture_mobile.h>
#include "tsid/robots/robot-wrapper.hpp"

#include <control_core/math.h>
#include <control_core/ros/parameters.h>

#include <ics_tsid_common/utilities/conversions.h>

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskJointPostureMobile::TaskJointPostureMobile(
        const std::string &name,
        RobotWrapper &robot) : TaskMotion(name, robot),
                               m_ref(robot.na()),
                               m_constraint(name, robot.na(), robot.nv())
    {
      m_Kp.setZero(robot.na());
      m_Kd.setZero(robot.na());
      Vector m = Vector::Ones(robot.na());
      setMask(m);
    }

    void TaskJointPostureMobile::setMask(ConstRefVector m)
    {
      assert(m.size() == m_robot.na());
      m_mask = m;
      const Vector::Index dim = static_cast<Vector::Index>(m.sum());
      Matrix S = Matrix::Zero(dim, m_robot.nv());
      m_activeAxes.resize(dim);
      unsigned int j = 0;
      for (unsigned int i = 0; i < m.size(); i++)
        if (m(i) != 0.0)
        {
          assert(m(i) == 1.0);
          S(j, m_robot.nv() - m_robot.na() + i) = 1.0;
          m_activeAxes(j) = i;
          j++;
        }
      m_constraint.resize((unsigned int)dim, m_robot.nv());
      m_constraint.setMatrix(S);
    }

    int TaskJointPostureMobile::dim() const
    {
      return (int)m_mask.sum();
    }

    const Vector &TaskJointPostureMobile::Kp() { return m_Kp; }

    const Vector &TaskJointPostureMobile::Kd() { return m_Kd; }

    void TaskJointPostureMobile::Kp(ConstRefVector Kp)
    {
      assert(Kp.size() == m_robot.na());
      m_Kp = Kp;
    }

    void TaskJointPostureMobile::Kd(ConstRefVector Kd)
    {
      assert(Kd.size() == m_robot.na());
      m_Kd = Kd;
    }

    void TaskJointPostureMobile::setReference(const TrajectorySample &ref)
    {
      assert(ref.getValue().size() == m_robot.na());
      assert(ref.getDerivative().size() == m_robot.na());
      assert(ref.getSecondDerivative().size() == m_robot.na());
      m_ref = ref;
    }

    const TrajectorySample &TaskJointPostureMobile::getReference() const
    {
      return m_ref;
    }

    const Vector &TaskJointPostureMobile::getDesiredAcceleration() const
    {
      return m_a_des;
    }

    Vector TaskJointPostureMobile::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix() * dv;
    }

    const Vector &TaskJointPostureMobile::position_error() const
    {
      return m_p_error;
    }

    const Vector &TaskJointPostureMobile::velocity_error() const
    {
      return m_v_error;
    }

    const Vector &TaskJointPostureMobile::position() const
    {
      return m_p;
    }

    const Vector &TaskJointPostureMobile::velocity() const
    {
      return m_v;
    }

    const Vector &TaskJointPostureMobile::position_ref() const
    {
      return m_ref.getValue();
    }

    const Vector &TaskJointPostureMobile::velocity_ref() const
    {
      return m_ref.getDerivative();
    }

    const ConstraintBase &TaskJointPostureMobile::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase &TaskJointPostureMobile::compute(const double,
                                                          ConstRefVector q,
                                                          ConstRefVector v,
                                                          Data &)
    {
      // [x,y,c,s,q] -> [x,y,theta,q]
      m_p.resize(m_robot.na());
      m_p.head(2) = q.head(2);
      m_p[2] = cc::cart_to_polar(q[3], q[2]);
      m_p.tail(m_robot.na() - 3) = q.tail(m_robot.na() - 3);

      m_v = v.tail(m_robot.na());
    
      // Compute errors
      m_p_error = m_p - m_ref.getValue();
      m_v_error = m_v - m_ref.getDerivative();
      m_a_des = -m_Kp.cwiseProduct(m_p_error) - m_Kd.cwiseProduct(m_v_error) + m_ref.getSecondDerivative();

      for (unsigned int i = 0; i < m_activeAxes.size(); i++)
        m_constraint.vector()(i) = m_a_des(m_activeAxes(i));
      return m_constraint;
    }

    std::shared_ptr<TaskBase> TaskJointPostureMobile::Load(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& name,
      bool activate,
      bool verbose)
    {
      cc::Parameters parameter(nh, name);
      parameter.addRequired<cc::VectorX>("kp");
      parameter.addOptional<cc::Scalar>("kd", -1);
      parameter.addRequired<cc::Scalar>("weight");
      parameter.addRequired<cc::VectorX>("posture");
      parameter.addRequired<cc::VectorX>("mask");
      if (!parameter.load())
      {
        ROS_ERROR("load_posture_task: Failed to load parameters of '%s'", name.c_str());
        return nullptr;
      }
      cc::VectorX kp = parameter.get<cc::VectorX>("kp");
      cc::Scalar kd = parameter.get<cc::Scalar>("kd");
      cc::Scalar weight = parameter.get<cc::Scalar>("weight");

      unsigned int priority = 1;
      auto task = std::make_shared<TaskJointPostureMobile>(name, tsid_wrapper.robotInterface().wrapper());
      task->Kp(kp);
      if(kd < 0)
        task->Kd(2.0*task->Kp().cwiseSqrt());
      else
        task->Kd(kd*cc::VectorX::Ones(task->Kd().size()));
      task->setMask(parameter.get<cc::VectorX>("mask"));

      // set reference
      task->setReference(ics::to_sample(parameter.get<cc::VectorX>("posture")));

      if(verbose)
        ROS_INFO("Adding Task name='%s' :\t priority=%u weight=%.1e kp=%.2f, kd=%.2f", name.c_str(), priority, weight, kp[0], task->Kd()[0]);
      tsid_wrapper.loadTask(name, task, weight, priority, activate);
      return task;
    }
  }
}
