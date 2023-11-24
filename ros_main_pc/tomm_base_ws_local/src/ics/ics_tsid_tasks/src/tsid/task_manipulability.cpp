#include "tsid/robots/robot-wrapper.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"

#include <ics_tsid_tasks/tsid/task_manipulability.h>

#include <control_core/ros/parameters.h>

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    double TaskManipulability::t_kinematics_derivatives_call = -1.0;

    TaskManipulability::TaskManipulability(const std::string & name,
                                           const std::string & start_joint_name,
                                           RobotWrapper & robot):
      Base(name, robot),
      m_constraint(name, robot.na(), robot.nv())
    {
      // get ids of kinematic subchain
      assert(m_robot.model().existJointName(start_joint_name));
      auto start_joint_id = m_robot.model().getJointId(start_joint_name);
      auto ids = robot.model().subtrees[start_joint_id];
      m_joint_id = ids.back();

      m_Kp.setZero(ids.size());
      m_Kd.setZero(ids.size());

      // mask all other joints except subchain
      Vector m = Vector::Zero(robot.na());
      for(auto id : ids)
      {
        auto idx_v = robot.model().joints[id].idx_v();
        m[idx_v - 6] = 1;
        m_indices_v.push_back(idx_v);
      }
      setMask(m);

      m_manipulability_thres = 0.01;

      m_hessian = Tensor3x(6, robot.nv(), robot.nv());
      m_hessian.setZero();
      m_J_full.resize(6, robot.nv());
    }

    void TaskManipulability::setMask(ConstRefVector m)
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
      m_constraint.resize((unsigned int)dim, m_robot.nv());
      m_constraint.setMatrix(S);

      m_J.resize(6, (unsigned int)dim);
      m_dm_dq.resize((unsigned int)dim);
    }

    void TaskManipulability::setManipulabilityThreshold(double manipulability_thres)
    {
      m_manipulability_thres = manipulability_thres;
    }

    int TaskManipulability::dim() const
    {
      return (int)m_mask.sum();
    }

    const Vector & TaskManipulability::Kp(){ return m_Kp; }

    const Vector & TaskManipulability::Kd(){ return m_Kd; }

    void TaskManipulability::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==m_robot.na());
      m_Kp = Kp;
    }

    void TaskManipulability::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==m_robot.na());
      m_Kd = Kd;
    }

    const Vector & TaskManipulability::getDesiredAcceleration() const
    {
      return m_a_des;
    }

    Vector TaskManipulability::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv;
    }

    const ConstraintBase & TaskManipulability::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskManipulability::compute(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
    {
      // full jacobian
      m_robot.jacobianLocal(data, m_joint_id, m_J_full);

      // sub chain jacobian
      m_J =  m_J_full * m_constraint.matrix().transpose();
      m_A = m_J*m_J.transpose();

      // manipulability index
      m_manip = std::sqrt(m_A.determinant());

      // check if need to do something
      m_dm_dq.setZero();
      if(m_manip < m_manipulability_thres)
      {
        // activate a repulsive action
        Matrix6 A_inv = m_A.inverse();

        // compute joint kinematic hessians (only once per timestep)
        if(t != t_kinematics_derivatives_call)
        {
          pinocchio::computeJointKinematicHessians(m_robot.model(), data, q);
          t_kinematics_derivatives_call = t;
        }

        // extract the hessian of the endeffector
        pinocchio::getJointKinematicHessian(m_robot.model(), data, m_joint_id, pinocchio::LOCAL, m_hessian);

        // compute the gradient based on reduced chain jacobian
        Eigen::DenseIndex outer_offset = m_robot.model().nv * 6;
        Matrix6 dA_dqi, dJ_dqi; 
        for(size_t i = 0; i < m_indices_v.size(); ++i)
        {
          int idx = m_indices_v[i];
          Eigen::Map<Data::Matrix6x> dJ_dqi_full(m_hessian.data() + idx * outer_offset, 6, m_robot.model().nv);
          dJ_dqi = dJ_dqi_full * m_constraint.matrix().transpose();

          dA_dqi = dJ_dqi*m_J.transpose() + m_J*dJ_dqi.transpose();

          m_dm_dq[i] = m_manip*(A_inv*dA_dqi).trace();
        }
      }
      
      m_v = m_constraint.matrix()*v.tail(m_robot.na());
      m_a_des = m_Kp.cwiseProduct(m_dm_dq)
                - m_Kd.cwiseProduct(m_v);

      m_constraint.vector() = m_a_des;
      return m_constraint;
    }

    std::shared_ptr<TaskBase> TaskManipulability::Load(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& name,
      bool activate,
      bool verbose)
    {
      cc::Parameters parameter(nh, name);
      parameter.addRequired<cc::VectorX>("kp");
      parameter.addOptional<cc::Scalar>("kd", -1);
      parameter.addRequired<std::string>("start_joint");
      parameter.addRequired<cc::Scalar>("weight");
      parameter.addRequired<cc::Scalar>("manipulability_threshold");
      if (!parameter.load())
      {
        ROS_ERROR("load_posture_task: Failed to load parameters of '%s'", name.c_str());
        return nullptr;
      }
      auto &robot = tsid_wrapper.robotInterface().wrapper();

      std::string start_joint_name = parameter.get<std::string>("start_joint");
      if(!robot.model().existJointName(start_joint_name))
      {
        ROS_ERROR("make_manipulability_task: joint '%s' does not exist", start_joint_name.c_str());
        return nullptr;
      }
      unsigned int priority = 1;
      auto task = std::make_shared<tsid::tasks::TaskManipulability>(name, start_joint_name, robot);

      auto weight = parameter.get<cc::Scalar>("weight");
      auto kp = parameter.get<cc::VectorX>("kp");
      auto kd = parameter.get<cc::Scalar>("kd");
      task->Kp(kp);
      if(kd < 0)
        task->Kd(2.0*task->Kp().cwiseSqrt());
      else
        task->Kd(kd*cc::VectorX::Ones(task->Kp().size()));

      if(verbose)
        ROS_INFO("Adding Task name='%s' :\t priority=%u weight=%.1e kp=%.2f, kd=%.2f", name.c_str(), priority, weight, kp[0], task->Kd()[0]);
      task->setManipulabilityThreshold(parameter.get<cc::Scalar>("manipulability_threshold"));
      
      tsid_wrapper.loadTask(name, task, weight, priority, activate);
      return task;
    }

  }
}