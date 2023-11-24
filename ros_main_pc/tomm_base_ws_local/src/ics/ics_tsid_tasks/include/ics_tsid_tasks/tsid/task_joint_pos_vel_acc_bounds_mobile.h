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

#ifndef __invdyn_task_joint_posVelAcc_bounds_mobile_hpp__
#define __invdyn_task_joint_posVelAcc_bounds_mobile_hpp__

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/tasks/task-motion.hpp>
#include <tsid/math/constraint-bound.hpp>
#include <tsid/math/constraint-inequality.hpp>
#include <tsid/deprecated.hh>

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

    class TaskJointPosVelAccBoundsMobile : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Vector Vector;
      typedef math::ConstraintBound ConstraintBound;
      typedef math::ConstraintInequality ConstraintInequality;
      typedef math::VectorXi VectorXi;
      typedef pinocchio::Data Data;

      static std::shared_ptr<TaskBase> Load(
          ics::TSIDWrapperInterface &tsid_wrapper,
          ros::NodeHandle &nh,
          const std::string &name,
          bool activate = true,
          bool verbose = true);
    
    public:
      TaskJointPosVelAccBoundsMobile(const std::string &name, RobotWrapper &robot,
                                     double dt, bool verbose = true);

      virtual ~TaskJointPosVelAccBoundsMobile() {}

      int dim() const;

      const ConstraintBase &compute(const double t, ConstRefVector q,
                                    ConstRefVector v, Data &data);

      const ConstraintBase &getConstraint() const;

      void setTimeStep(double dt);
      void setPositionBounds(ConstRefVector lower, ConstRefVector upper);
      void setVelocityBounds(ConstRefVector upper);
      void setAccelerationBounds(ConstRefVector upper);
      const Vector &getAccelerationBounds() const;
      const Vector &getVelocityBounds() const;
      const Vector &getPositionLowerBounds() const;
      const Vector &getPositionUpperBounds() const;

      void setVerbose(bool verbose);

      void setImposeBounds(bool impose_position_bounds, bool impose_velocity_bounds,
                           bool impose_viability_bounds,
                           bool impose_acceleration_bounds);

      void isStateViable(ConstRefVector q, ConstRefVector dq, bool verbose = true);

      void computeAccLimitsFromPosLimits(ConstRefVector q, ConstRefVector dq,
                                         bool verbose = true);

      void computeAccLimitsFromViability(ConstRefVector q, ConstRefVector dq,
                                         bool verbose = true);

      void computeAccLimits(ConstRefVector q, ConstRefVector dq,
                            bool verbose = true);

      TSID_DEPRECATED const Vector &mask() const;    // deprecated
      TSID_DEPRECATED void mask(const Vector &mask); // deprecated
      virtual void setMask(math::ConstRefVector mask);

    protected:
      ConstraintInequality m_constraint;
      double m_dt;
      bool m_verbose;
      int m_nv, m_na;

      Vector m_mask;
      VectorXi m_activeAxes;

      Vector m_qa;  // actuated part of q
      Vector m_dqa; // actuated part of dq

      double m_eps; // tolerance used to check violations

      Vector m_p; // compressed joint position

      Vector m_qMin;   // joints position limits
      Vector m_qMax;   // joints position limits
      Vector m_dqMax;  // joints max velocity limits
      Vector m_ddqMax; // joints max acceleration limits

      Vector m_dqMinViab; // velocity lower limits from viability
      Vector m_dqMaxViab; // velocity upper limits from viability

      Vector m_ddqLBPos; // acceleration lower bound from position bounds
      Vector m_ddqUBPos; // acceleration upper bound from position bounds
      Vector m_ddqLBVia; // acceleration lower bound from viability bounds
      Vector m_ddqUBVia; // acceleration upper bound from viability bounds
      Vector m_ddqLBVel; // acceleration lower bound from velocity bounds
      Vector m_ddqUBVel; // acceleration upper bound from velocity bounds
      Vector m_ddqLBAcc; // acceleration lower bound from acceleration bounds
      Vector m_ddqUBAcc; // acceleration upper bound from acceleration bounds

      Vector m_ddqLB; // final acceleration bounds
      Vector m_ddqUB; // final acceleration bounds

      bool m_impose_position_bounds;
      bool m_impose_velocity_bounds;
      bool m_impose_viability_bounds;
      bool m_impose_acceleration_bounds;

      Vector m_viabViol; // 0 if the state is viable, error otherwise

      // Used in computeAccLimitsFromPosLimits
      double m_two_dt_sq;
      Vector m_ddqMax_q3;
      Vector m_ddqMin_q3;
      Vector m_ddqMax_q2;
      Vector m_ddqMin_q2;
      Vector m_minus_dq_over_dt;

      // Used in computeAccLimitsFromViability
      double m_dt_square;
      Vector m_dt_dq;
      Vector m_dt_two_dq;
      Vector m_two_ddqMax;
      Vector m_dt_ddqMax_dt;
      Vector m_dq_square;
      Vector m_q_plus_dt_dq;
      double m_two_a;
      Vector m_b_1;
      Vector m_b_2;
      Vector m_ddq_1;
      Vector m_ddq_2;
      Vector m_c_1;
      Vector m_delta_1;
      Vector m_c_2;
      Vector m_delta_2;

      // Used in computeAccLimits
      Vector m_ub;
      Vector m_lb;
    };

  } // namespace tasks
} // namespace tsid

#endif // ifndef __invdyn_task_joint_bounds_hpp__