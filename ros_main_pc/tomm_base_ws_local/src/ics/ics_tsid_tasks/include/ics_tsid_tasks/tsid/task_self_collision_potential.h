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
//
// See: https://github.com/resibots/inria_wbc

#ifndef ICS_TSID_TASKS_invdyn_task_self_collision_potential_hpp__
#define ICS_TSID_TASKS_invdyn_task_self_collision_potential_hpp__

#include <unordered_map>

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"

////////////////////////////////////////////////////////////////////////////////
// pinocchio includes
////////////////////////////////////////////////////////////////////////////////
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

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

    class TaskSelfCollisionPotential : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef math::Vector Vector;
      typedef math::Vector3 Vector3;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data::Matrix6x Matrix6x;

      static std::shared_ptr<TaskBase> Load(
        ics::TSIDWrapperInterface& tsid_wrapper,
        ros::NodeHandle& nh,
        const std::string& name,
        bool activate = true,
        bool verbose = true);

    public:
      /**
       * @brief Construct a new Task Self Collision object
       * 
       * Self collision avoidance based on potential fields. 
       * Dosn't work well, better use CollisionAvoidanceConstraint task
       * 
       * @param name task name
       * @param robot robot
       * @param frameName tracked frame name
       * @param frames collision frames
       * @param radius minimum radius before collision
       * @param radius_max maximum radius for avoidance
       * @param c_max maximum potential (at collision distance)
       * @param c_min minimum potential (at radius_max)
       */
      TaskSelfCollisionPotential(const std::string &name,
                        RobotWrapper &robot,
                        const std::string &frameName,
                        const std::unordered_map<std::string, double> &frames,
                        double radius,
                        double influence_zone,
                        double c_max=1.0,
                        double c_min=1e-3);
      virtual ~TaskSelfCollisionPotential() {}

      /**
       * @brief check if there is any collision between tracked an collsion frames
       * 
       * @return true 
       * @return false 
       */
      bool hasCollision(std::string* name=NULL, double* distance=NULL) const;

      int dim() const;

      const ConstraintBase &compute(const double t,
                                    ConstRefVector q,
                                    ConstRefVector v,
                                    Data &data);

      const ConstraintBase &getConstraint() const;

      double Kp() const { return m_Kp; }
      double Kd() const { return m_Kd; }

      void Kp(const double kp) { m_Kp = kp; }
      void Kd(const double kd) { m_Kd = kd; }
      Index frame_id() const;

      const std::vector<Vector3> &avoided_frames_positions() const { return m_avoided_frames_positions; }
      const std::vector<double> &avoided_frames_r0s() const { return m_avoided_frames_r0s; }
      double radius() const { return m_radius; }

    protected:
      bool compute_C(const Vector3 &x, const std::vector<Vector3> &frames_positions);
      void compute_grad_C(const Vector3 &x, const std::vector<Vector3> &frames_positions);
      void compute_Hessian_C(const Vector3 &x, const std::vector<Vector3> &frames_positions);

      std::string m_tracked_frame_name;                               // name of the body that we track
      Index m_tracked_frame_id;                                       // id of the body that we track (from the model)
      std::unordered_map<std::string, double> m_avoided_frames_names; // names of the bodies to avoid and radius
      std::vector<Index> m_avoided_frames_ids;                        // id of the bodies to avoid
      std::vector<double> m_avoided_frames_r0s;                       // radius for each avoided frame
      std::vector<double> m_exponents_p;                              // exponent for decay of each avoided frame

      double m_Kp;
      double m_Kd;

      Eigen::Matrix<double, 1, 1> m_C;
      Vector3 m_grad_C;
      Eigen::Matrix<double, 3, 3> m_Hessian_C;

      double m_radius;
      double m_influence_zone;
      double m_c_scale;
      double m_c_min;
      ConstraintEquality m_constraint;

      Vector3 m_drift;
      Matrix6x m_J;               // jacobian of the tracked frame
      std::vector<Matrix6x> m_Js; // jacobian of the other frames

      Vector3 tracked_frame_position;
      std::vector<Vector3> m_avoided_frames_positions;

      std::vector<bool> m_collisions;

      Eigen::MatrixXd m_A;
      Eigen::VectorXd m_B;
    };

  } // namespace tasks
} // namespace tsid

#endif // ifndef __invdyn_task_pos_avoidance_hpp__
