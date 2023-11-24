//
// Copyright (c) 2017-2020 CNRS, Inria
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

//#define DEBUG_POS_AVOIDANCE
#include <cmath>

#include <tsid/robots/robot-wrapper.hpp>

#include <ics_tsid_tasks/tsid/task_self_collision_potential.h>

#include <control_core/ros/parameters.h>

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskSelfCollisionPotential::TaskSelfCollisionPotential(const std::string &name,
                                         RobotWrapper &robot,
                                         const std::string &tracked_frame_name,
                                         const std::unordered_map<std::string, double> &avoided_frames_names,
                                         double radius,
                                         double influence_zone,
                                         double c_max,
                                         double c_min)
        : TaskMotion(name, robot),
          m_tracked_frame_name(tracked_frame_name),
          m_avoided_frames_names(avoided_frames_names),
          m_radius(radius),
          m_influence_zone(influence_zone),
          m_c_scale(c_max/exp(-1)),
          m_c_min(c_min),
          m_constraint(name, 1, robot.nv()),
          m_Js(avoided_frames_names.size()),
          m_avoided_frames_positions(avoided_frames_names.size())

    {
      assert(m_robot.model().existFrame(m_tracked_frame_name));
      m_tracked_frame_id = m_robot.model().getFrameId(m_tracked_frame_name);
      m_Kp = 0.;
      m_Kd = 0.;
      m_grad_C.setZero(3);
      m_J.setZero(6, robot.nv());
      for (size_t i = 0; i < m_Js.size(); ++i)
        m_Js[i].setZero(6, robot.nv());

      for (const auto &it : m_avoided_frames_names)
      {
        assert(m_robot.model().existFrame(it.first));
        // save the frame id + min radius
        m_avoided_frames_ids.push_back(m_robot.model().getFrameId(it.first));
        m_avoided_frames_r0s.push_back(it.second);
        // save the exponent
        double a0 = it.second + m_radius;
        double a1 = a0 + influence_zone;
        double p = log(1 - log(m_c_min)) / log(a1/a0);
        m_exponents_p.push_back(p);
      }
      m_collisions.resize(m_avoided_frames_ids.size());
    }

    int TaskSelfCollisionPotential::dim() const
    {
      return 1;
    }

    bool TaskSelfCollisionPotential::hasCollision(std::string* name, double* distance) const 
    {
      auto it = std::find(m_collisions.begin(), m_collisions.end(), true);
      if(it == m_collisions.end())
        return false;

      auto idx = std::distance(m_collisions.begin(), it);

      if(name) 
      {
        auto frame_it = m_avoided_frames_names.begin();
        std::advance(frame_it, idx);
        *name = frame_it->first;
      }
      if(distance)
      {
        *distance = (tracked_frame_position - m_avoided_frames_positions[idx]).norm();
      }
      return true;
    }

    const ConstraintBase &TaskSelfCollisionPotential::getConstraint() const
    {
      return m_constraint;
    }

    Index TaskSelfCollisionPotential::frame_id() const
    {
      return m_tracked_frame_id;
    }

    const ConstraintBase &TaskSelfCollisionPotential::compute(const double,
                                                     ConstRefVector q,
                                                     ConstRefVector v,
                                                     Data &data)
    {
      // pos & Jacobian of the tracked frame
      SE3 oMi;
      Motion v_frame;
      Motion a_frame;
      m_robot.framePosition(data, m_tracked_frame_id, oMi);
      //a_frame = m_robot.frameAccelerationWorldOriented(data, m_tracked_frame_id);
      m_robot.frameClassicAcceleration(data, m_tracked_frame_id, a_frame);
      //a_frame = data.a[m_tracked_frame_id];

      tracked_frame_position = oMi.translation();
      m_drift = a_frame.linear();
      m_robot.frameJacobianWorld(data, m_tracked_frame_id, m_J);
      //m_robot.frameJacobianLocal(data, m_tracked_frame_id, m_J);

      auto J1 = m_J.block(0, 0, 3, m_robot.nv());

      // collision space is one dimensional (line between two bodies)
      m_A = Eigen::MatrixXd::Zero(1, m_robot.nv());
      m_B = Eigen::MatrixXd::Zero(1, 1);

      // keep track of collisions
      std::fill(m_collisions.begin(), m_collisions.end(), false);

      static const Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);

      const Vector3& pos = tracked_frame_position;
      for (size_t i = 0; i < m_avoided_frames_ids.size(); ++i)
      {
        // pos & Jacobian
        m_robot.framePosition(data, m_avoided_frames_ids[i], oMi);
        m_avoided_frames_positions[i] = oMi.translation();

        // distance with tracked frame
        const Vector3& pos2 = m_avoided_frames_positions[i];
        Vector3 diff = pos - pos2;
        double square_norm = diff.dot(diff);
        double norm = sqrt(square_norm);
        double r = m_avoided_frames_r0s[i];
        double p = m_exponents_p[i];
        double a0 = (r + m_radius);
        double a1 = a0 + m_influence_zone;

        // if in the collision zone
        if (norm <= a0)
        {
          m_collisions[i] = true;
        }

        // in the influence zone
        if(norm <= a1)
        {
          // std::cout << "activated for:" << m_tracked_frame_name << " norm:" << norm << " r:" << r << " radius:" << m_radius << std::endl;
          m_robot.frameJacobianWorld(data, m_avoided_frames_ids[i], m_Js[i]);
          //a_frame = m_robot.frameAccelerationWorldOriented(data, m_avoided_frames_ids[i]);
          m_robot.frameClassicAcceleration(data, m_avoided_frames_ids[i], a_frame); // TODO dJ.dq in world frame
          //a_frame = data.a[m_avoided_frames_ids[i]];
          auto J = J1 - m_Js[i].block(0, 0, 3, m_robot.nv());
          Eigen::Vector3d drift = m_drift - a_frame.linear();

          // potential, gradient and hessian
          m_C(0, 0) = m_c_scale*exp(-pow(norm / a0, p));
          m_grad_C = -(p / pow(a0, p) * pow(norm, p - 2)) * m_C(0, 0) * diff;
          m_Hessian_C = m_C(0, 0) * ((p * p / pow(a0, 2 * p) * pow(norm, 2 * p - 4) - p * (p - 2) / pow(a0, p) * pow(norm, p - 4)) * diff * diff.transpose() - (p / pow(a0, p) * pow(norm, p - 2)) * I);

          // A
          m_A += m_grad_C.transpose() * J;
          // B (note: m_drift = dJ(q)*dq)
          // m_B += -((m_Hessian_C * J * v).transpose() * J * v + m_grad_C.transpose() * (-drift + m_Kd * J * v) + m_Kp * m_C);
          m_B += -(m_grad_C.transpose() * (-drift + m_Kd * J * v) + m_Kp * m_C);
        } // else 0 for everything
      }

      m_constraint.setMatrix(m_A);
      m_constraint.setVector(m_B);
      return m_constraint;
    }

    std::shared_ptr<TaskBase> TaskSelfCollisionPotential::Load(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& name,
      bool activate,
      bool verbose)
    {
      cc::Parameters parameter(nh, name);
      parameter.addRequired<std::string>("frame");
      parameter.addRequired<cc::Scalar>("kp");
      parameter.addOptional<cc::Scalar>("kd", -1);
      parameter.addRequired<cc::Scalar>("weight");
      parameter.addRequired<cc::Scalar>("radius");
      parameter.addRequired<cc::Scalar>("influence_zone");
      parameter.addOptional<cc::Scalar>("c_max", 1.0);
      parameter.addOptional<cc::Scalar>("c_min", 1e-3);
      parameter.addRequired<std::vector<std::string> >("avoided_frames");
      parameter.addRequired<std::vector<cc::Scalar> >("avoided_radius");
      if (!parameter.load())
      {
        ROS_ERROR("load_self_collision_task: Failed to load parameters of '%s'", name.c_str());
        return nullptr;
      }

      auto avoided_frames = parameter.get<std::vector<std::string> >("avoided_frames");
      auto avoided_radius = parameter.get<std::vector<cc::Scalar> >("avoided_radius");
      if(avoided_frames.size() != avoided_radius.size())
      {
        ROS_ERROR("load_self_collision_task: 'avoided_frames' not matching 'avoided_radius'");
        return nullptr;
      }
      std::unordered_map<std::string, double> avoided;
      for(size_t i = 0; i < avoided_frames.size(); ++i)
      {
        avoided[avoided_frames[i]] = avoided_radius[i];
      }

      for(const auto& avoided_frame : avoided)
      {
        if(!tsid_wrapper.robotInterface().wrapper().model().existFrame(avoided_frame.first))
        {
          ROS_ERROR("make_se3_task: frame '%s' does not exist", avoided_frame.first.c_str());
          return nullptr;
        }
      }

      // build task 
      unsigned int priority = 1;
      auto task = std::make_shared<tsid::tasks::TaskSelfCollisionPotential>(
        name, tsid_wrapper.robotInterface().wrapper(), parameter.get<std::string>("frame"), 
        avoided, parameter.get<cc::Scalar>("radius"), 
        parameter.get<cc::Scalar>("influence_zone"), 
        parameter.get<cc::Scalar>("c_max"), parameter.get<cc::Scalar>("c_min"));

      cc::Scalar weight = parameter.get<cc::Scalar>("weight");
      cc::Scalar kp = parameter.get<cc::Scalar>("kp");
      cc::Scalar kd = parameter.get<cc::Scalar>("kd");
      if(kd<0)
        kd = 2.0*std::sqrt(kp);
      task->Kp(kp);
      task->Kd(kd);

      if(verbose)
      {
        ROS_INFO("Adding Self Collision name='%s' :\t priority=%u weight=%.1e, kp=%.2f, kd=%.2f", name.c_str(), priority, weight, kp, kd);
        for(const auto& elem : avoided)
        {
          ROS_INFO("- frame='%s' : radius=%.2f", elem.first.c_str(), elem.second);
        }
      }
      tsid_wrapper.loadTask(name, task, weight, priority, activate);
      return task;
    }

  } // namespace tasks
} // namespace tsid
