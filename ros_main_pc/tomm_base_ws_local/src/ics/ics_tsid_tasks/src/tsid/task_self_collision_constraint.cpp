#include "tsid/robots/robot-wrapper.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include <ics_tsid_tasks/tsid/task_self_collision_constraint.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include <control_core/math/adjoint.h>
#include <control_core/ros/parameters.h>
#include <control_core/test_utilities/timing.h>

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskSelfCollisionConstraint::TaskSelfCollisionConstraint(const std::string & name,
                                                             cc::Scalar dt,
                                                             cc::Scalar v_damp,
                                                             cc::Scalar d_min,
                                                             cc::Scalar d_start,
                                                             GeometryModel & geometry_model,
                                                             GeometryData & geometry_data,
                                                             RobotWrapper & robot):
      Base(name, robot),
      m_dt(dt),
      m_v_max(v_damp),
      m_d_min(d_min),
      m_d_start(d_start),
      m_geometry_model(geometry_model),
      m_geometry_data(geometry_data),
      m_constraint(name)
    {
      m_eps = 1e-5;
      t_prev_vis = 0.0;

      // setup the security margin
      pinocchio::GeometryData::MatrixXs security_margin_map(
        pinocchio::GeometryData::MatrixXs::Constant(m_geometry_model.ngeoms,m_geometry_model.ngeoms, m_d_start));
      m_geometry_data.setSecurityMargins(m_geometry_model, security_margin_map);

      J_c1.setZero(6, robot.nv());                                              
      J_c2.setZero(6, robot.nv());
      J_12.setZero(1, robot.nv());

      R_1_w.setIdentity(); 
      R_2_w.setIdentity();
      X_c1_b.setIdentity(); 
      X_c2_b.setIdentity();

      num_active_ieq_ = 0;
      num_violated_ieq_ = 0;
      min_distance_ = 1e10;

      size_t n_collision_pairs = m_geometry_model.collisionPairs.size();
      m_constraint.matrix().setZero(n_collision_pairs, robot.nv());
      m_constraint.lowerBound().setConstant(n_collision_pairs, -1e10);
      m_constraint.upperBound().setConstant(n_collision_pairs, 1e10);

      Vector m = Vector::Ones(robot.na());
      setMask(m);
      
      // setup ros visualizations
      distance_pt_marker.id = 0;
      distance_pt_marker.ns = "distance_pt";
      distance_pt_marker.header.frame_id = "world";
      distance_pt_marker.type = visualization_msgs::Marker::SPHERE_LIST;
      distance_pt_marker.color.r = 0.0;
      distance_pt_marker.color.g = 1.0;
      distance_pt_marker.color.b = 0.0;
      distance_pt_marker.color.a = 1.0;
      distance_pt_marker.scale.x = 0.02;
      distance_pt_marker.pose.orientation.w = 1.0;

      distance_n_marker.id = 1;
      distance_n_marker.ns = "distance_n";
      distance_n_marker.header.frame_id = "world";
      distance_n_marker.type = visualization_msgs::Marker::LINE_LIST;
      distance_n_marker.color.r = 0.0;
      distance_n_marker.color.g = 1.0;
      distance_n_marker.color.b = 0.0;
      distance_n_marker.color.a = 1.0;
      distance_n_marker.scale.x = 0.01;

      ros::NodeHandle nh;
      distance_pt_pub = nh.advertise<visualization_msgs::Marker>(
        "/distance_pt", 1);
      distance_n_pub = nh.advertise<visualization_msgs::Marker>(
        "/distance_n", 1);
    }

    bool TaskSelfCollisionConstraint::connect(ros::NodeHandle& nh)
    {
      //////////////////////////////////////////////////////////////////////////
      // debug msg
      //////////////////////////////////////////////////////////////////////////
      msg_pub_ = nh.advertise<ics_tsid_task_msgs::SelfCollisionConstraint>(Base::name() + "_debug", 1);
      return true;
    }

    int TaskSelfCollisionConstraint::dim() const
    {
      return (int)m_mask.sum();
    }

    void TaskSelfCollisionConstraint::setMask(ConstRefVector m)
    {
      assert(m.size()==m_robot.na());
      m_mask = m;
      const Vector::Index dim = static_cast<Vector::Index>(m.sum());
      Matrix S = Matrix::Zero(dim, m_robot.nv());
      m_activeAxes.resize(dim);
      unsigned int j=0;
      for(unsigned int i=0; i<m.size(); i++)
        if(m(i)!=0.0)
        {
          assert(m(i)==1.0);
          S(j,m_robot.nv()-m_robot.na()+i) = 1.0;
          m_activeAxes(j) = i;
          j++;
        }
    }

    const ConstraintBase & TaskSelfCollisionConstraint::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskSelfCollisionConstraint::compute(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
    {      
      // reset states
      num_active_ieq_ = 0;
      num_violated_ieq_ = 0;
      min_distance_ = 1e10;

      // reset msg
      msg_.distances.data.clear();

      //////////////////////////////////////////////////////////////////////////
      // iterate over all collision pairs
      //////////////////////////////////////////////////////////////////////////
      auto start_time = TIMENOW();
      for(size_t cp_index = 0; cp_index < m_geometry_model.collisionPairs.size(); ++cp_index)
      {
        const auto& cp = m_geometry_model.collisionPairs[cp_index];
        const pinocchio::GeometryObject& obj1 = m_geometry_model.geometryObjects[cp.first];
        const pinocchio::GeometryObject& obj2 = m_geometry_model.geometryObjects[cp.second];
        const hpp::fcl::CollisionResult& res = m_geometry_data.collisionResults[cp_index];

        if(res.isCollision())
        {
          // compute an accurate distance
          pinocchio::computeDistance(m_geometry_model, m_geometry_data, cp_index);
          hpp::fcl::DistanceResult& dres = m_geometry_data.distanceResults[cp_index];
          auto contact = res.getContact(0);

          // contact normal in the world frame
          n_12 = -(dres.nearest_points[0] -  dres.nearest_points[1]).cast<double>().normalized();

          // transformations joint wrt world
          X_1_w = m_robot.position(data, obj1.parentJoint);
          R_1_w.rotation(X_1_w.rotation()); 

          X_2_w = m_robot.position(data, obj2.parentJoint);
          R_2_w.rotation(X_2_w.rotation()); 

          // transformations contact point wrt body
          X_c1_b.translation(
            R_1_w.rotation().transpose()*(dres.nearest_points[0].cast<double>() - X_1_w.translation()));
          X_c2_b.translation(
            R_2_w.rotation().transpose()*(dres.nearest_points[1].cast<double>() - X_2_w.translation()));

          // acceleration bias at the contact point, body rotated
          drift_1_b = X_c1_b.actInv(data.a[obj1.parentJoint]);
          v_joint = X_c1_b.actInv(data.v[obj1.parentJoint]);
          drift_1_b.linear() += v_joint.angular().cross(v_joint.linear());

          drift_2_b = X_c1_b.actInv(data.a[obj2.parentJoint]);
          v_joint = X_c1_b.actInv(data.v[obj2.parentJoint]);
          drift_2_b.linear() += v_joint.angular().cross(v_joint.linear());

          // compute jacobian at the contact point
          J_c1.setZero();
          placement = data.oMi[obj1.parentJoint] * X_c1_b;
          details::translateJointJacobian(m_robot.model(), data, obj1.parentJoint, pinocchio::LOCAL, placement, data.J, J_c1);

          J_c2.setZero();
          placement = data.oMi[obj2.parentJoint] * X_c2_b;
          details::translateJointJacobian(m_robot.model(), data, obj2.parentJoint, pinocchio::LOCAL, placement, data.J, J_c2);

          // rotate drift acceleration to world frame
          drift1 = R_1_w.act(drift_1_b);
          drift2 = R_1_w.act(drift_2_b);

          // rotate jacobian to world
          J_c1 = R_1_w.toActionMatrix() * J_c1;
          J_c2 = R_2_w.toActionMatrix() * J_c2;

          // collision space mapping: normal vector obj1 to obj2
          J_12 = n_12.transpose() * (J_c1.topRows(3) - J_c2.topRows(3));
          J_12.leftCols(6).setZero();

          // acceleration bias
          cc::Scalar drift12 = n_12.dot(drift1.head(3) - drift2.head(3));

          // compute inside the 1 dim collision space (note: d_12P > 0 approaching)
          cc::Scalar d_12 = dres.min_distance;
          cc::Scalar d_12P = J_12.dot(v);

          if(d_12 < min_distance_)
            min_distance_ = d_12;

          // activation of the constraint (note: 0 at m_d_min)
          cc::Scalar fac = (d_12 - m_d_min) / (m_d_start - m_d_min);
          cc::Scalar vel_limit = fac*m_v_max;
          cc::Scalar acc_limit = (vel_limit - d_12P) / m_dt;

          // setup the matrix
          // For tsid we have the form lb <= A*qPP <= ub
          m_constraint.matrix().row(cp_index) = J_12;
          m_constraint.upperBound()[cp_index] = acc_limit - drift12;
          
          // count constraints, save msg
          msg_.distances.data.push_back(d_12P);
          
          if(d_12 < (m_d_min - 1e-4))
            num_violated_ieq_++;
          num_active_ieq_++;
        }
        else
        {
          // back to maximum
          m_constraint.upperBound()[cp_index] = 1e10;
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // publish debugging information
      //////////////////////////////////////////////////////////////////////////
      msg_.dur.data = DURATION(start_time);
      msg_.num_active_ieq.data = num_active_ieq_;
      msg_.num_violated_ieq.data = num_violated_ieq_;
      msg_.min_distance.data = min_distance_;
      cc::publish_if_subscribed(msg_pub_, msg_);

      if(t - t_prev_vis > 1./30.)
      {
        t_prev_vis = t;
        visualize();
      }
      // ROS_INFO_STREAM_THROTTLE(0.5, "TaskSelfCollisionConstraint: d_min=" << m_min_distance);

      return m_constraint;
    }

    void TaskSelfCollisionConstraint::visualize()
    {
      geometry_msgs::Point point, point2;
      distance_n_marker.points.clear();
      distance_pt_marker.points.clear();
      for(size_t cp_index = 0; cp_index < m_geometry_model.collisionPairs.size(); ++cp_index)
      {
        const hpp::fcl::CollisionResult& res = m_geometry_data.collisionResults[cp_index];
        if(res.isCollision())
        {
          hpp::fcl::DistanceResult& dres = m_geometry_data.distanceResults[cp_index];
          point.x = dres.nearest_points[0].x();
          point.y = dres.nearest_points[0].y();
          point.z = dres.nearest_points[0].z();
          point2.x = dres.nearest_points[1].x();
          point2.y = dres.nearest_points[1].y();
          point2.z = dres.nearest_points[1].z();
          distance_pt_marker.points.push_back(point);
          distance_pt_marker.points.push_back(point2);
          distance_n_marker.points.push_back(point);
          distance_n_marker.points.push_back(point2);
        }
      }
      distance_pt_pub.publish(distance_pt_marker);
      distance_n_pub.publish(distance_n_marker);
    }

    std::shared_ptr<TaskBase> TaskSelfCollisionConstraint::Load(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& name,
      bool activate,
      bool verbose)
    {
      cc::Parameters parameter(nh, name);
      parameter.addRequired<cc::Scalar>("dt");
      parameter.addRequired<cc::Scalar>("v_damp");
      parameter.addRequired<cc::Scalar>("d_min");
      parameter.addRequired<cc::Scalar>("d_start");
      parameter.addRequired<cc::Scalar>("weight");
      if (!parameter.load())
      {
        ROS_ERROR("load_self_collision_constraint_task: Failed to load parameters of '%s'", name.c_str());
        return nullptr;
      }
      unsigned int priority = (parameter.get<cc::Scalar>("weight") <= 0) ? 0 : 1;
      auto task = std::make_shared<tsid::tasks::TaskSelfCollisionConstraint>(
        name, parameter.get<cc::Scalar>("dt"), parameter.get<cc::Scalar>("v_damp"), 
        parameter.get<cc::Scalar>("d_min"), parameter.get<cc::Scalar>("d_start"), 
        tsid_wrapper.robotInterface().geometryModel(), tsid_wrapper.geometryData(), tsid_wrapper.robotInterface().wrapper());
      if(!task->connect(nh))
      {
        ROS_ERROR("Failed to connect task='%s'", name.c_str());
        return nullptr;
      }
      
      if(verbose)
        ROS_INFO("Adding Task name='%s' :\t priority=%u", name.c_str(), priority);
      tsid_wrapper.loadTask(name, task, parameter.get<cc::Scalar>("weight"), priority, activate);
      return task;
    }
  }
}