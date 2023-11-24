#ifndef ICS_TSID_TASKS_invdyn_task_manipulability_hpp__
#define ICS_TSID_TASKS_invdyn_task_manipulability_hpp__

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
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types.h>
#include <control_core_msgs/SkinPatches.h>

////////////////////////////////////////////////////////////////////////////////
// ros includes
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>

namespace tsid
{
  namespace tasks
  {
    
    /**
     * @brief TaskManipulability
     * 
     * Computes the manipulability index m = sqrt(det(J*J^T)) of 
     * a kinematic chain (e.g. arms) and activates a potential that avoids
     * low manipulabiliy regions.
     * 
     * The gradient requires the computation of dJ/dq which is a tensor of 
     * third order [dJ/dq1,...,dJ/dqn].
     * The derivative is described in [matrix_cookbook.pdf] eq. 46
     */
    class TaskManipulability : public TaskMotion
    {
    public:
      typedef TaskMotion Base;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::VectorXi VectorXi;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;
      typedef pinocchio::Data::Matrix6x Matrix6x;
      typedef pinocchio::Data::Matrix6 Matrix6;
      typedef pinocchio::Data::Tensor3x Tensor3x;

      static std::shared_ptr<TaskBase> Load(
        ics::TSIDWrapperInterface& tsid_wrapper,
        ros::NodeHandle& nh,
        const std::string& name,
        bool activate = true,
        bool verbose = true);

    public:
      TaskManipulability(const std::string & name,
                         const std::string & start_joint_name,
                         RobotWrapper & robot);

      int dim() const;

      bool subscribe(ros::NodeHandle& nh);

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      const Vector & getDesiredAcceleration() const;
      Vector getAcceleration(ConstRefVector dv) const;

      virtual void setMask(math::ConstRefVector mask);

      void setManipulabilityThreshold(double manipulability_thres);

      const Vector & Kp();
      const Vector & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

    protected:
      static double t_kinematics_derivatives_call;

      double m_manipulability_thres;

      Index m_joint_id;                   // joint id of endeffector
      std::vector<int> m_indices_v;     // joint id of all chain joints

      Matrix6x m_J_full;                       // full jacobian
      Matrix6x m_J;                 // chain Jacobian
      Matrix6 m_A;                        // chain Jacobian 
      Tensor3x m_hessian;                 // kinematic hessian

      Vector m_Kp;
      Vector m_Kd;

      double m_manip;                     // manipulability
      Vector m_dm_dq;                     // manipulability gradient

      Vector m_v;
      Vector m_a_des;
      VectorXi m_activeAxes;
      ConstraintEquality m_constraint;
    };
    
  }
}

#endif