#ifndef ICS_TSID_TASKS_invdyn_task_skin_distance_constraint_hard_hpp__
#define ICS_TSID_TASKS_invdyn_task_skin_distance_constraint_hard_hpp__

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/tasks/task-motion.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/math/constraint-inequality.hpp>

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/interfaces/tsid_wrapper_interface.h>
#include <ics_tsid_tasks/TaskSkinDistanceConstraintsParameters.h>

////////////////////////////////////////////////////////////////////////////////
// ros includes
////////////////////////////////////////////////////////////////////////////////
#include <unordered_map>
#include <ros/ros.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types.h>

////////////////////////////////////////////////////////////////////////////////
// skin_client includes
////////////////////////////////////////////////////////////////////////////////
#include <skin_client/patch_client_factory.h>

////////////////////////////////////////////////////////////////////////////////
// qp solver includes
////////////////////////////////////////////////////////////////////////////////
#include <eiquadprog/eiquadprog-fast.hpp>

namespace tsid
{
  namespace tasks
  {
    
    /**
     * @brief TaskSkinDistanceConstraint
     * 
     * Reads the cell distances and adds them as 1 dim inequality contraints
     * to the formulation. 
     * 
     * @note This task should be added as a constraint, not a cost
     */
    class TaskSkinDistanceConstraintHard : public TaskMotion
    {
    public:
      typedef TaskMotion Base;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef math::Vector Vector;
      typedef math::ConstraintInequality ConstraintInequality;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;
      typedef pinocchio::Data::Matrix6 Matrix6;
      typedef pinocchio::Data::Matrix6x Matrix6x;

      typedef ics_tsid_tasks::TaskSkinDistanceConstraintsConfig Config;
      typedef ics_tsid_tasks::TaskSkinDistanceConstraintsServer Server;
      typedef ics_tsid_tasks::TaskSkinDistanceConstraintsParameters Parameters;

      typedef skin_client::SkinPatchClient::Ptr Client;
      typedef std::vector<Client> Clients;

      static std::shared_ptr<TaskBase> Load(
        ics::TSIDWrapperInterface& tsid_wrapper,
        ros::NodeHandle& nh,
        const std::string& name,
        bool activate = true,
        bool verbose = true);

    public:
      TaskSkinDistanceConstraintHard(const std::string & name,
                         cc::Scalar dt,
                         RobotWrapper & robot);

      bool connect(ros::NodeHandle& nh);

      int dim() const { return (int)m_mask.sum(); }

      double minimumDistance() const;

      int numActiveConstraints() const;

      virtual const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data) override;

      const ConstraintBase & getConstraint() const { return constraint_; }

      virtual void setMask(math::ConstRefVector mask);

    private:
        void reconfigureRequest(Config &config, uint32_t level);

    protected:
      Parameters params_;
      std::unique_ptr<Server> reconfig_srv_;

      //////////////////////////////////////////////////////////////////////////
      // settings
      //////////////////////////////////////////////////////////////////////////
      double dt_;
      double min_dist_;

      int num_active_cell_;
      int num_active_ieq_;
      int num_active_eq_;
      
      //////////////////////////////////////////////////////////////////////////
      // skin clients
      //////////////////////////////////////////////////////////////////////////
      Clients clients_;
      std::vector<size_t> m_client_ids;
      
      //////////////////////////////////////////////////////////////////////////
      // transformations
      //////////////////////////////////////////////////////////////////////////
      pinocchio::SE3 X_c_b;
      pinocchio::SE3 placement;
      pinocchio::Motion v_joint, drift_b;
      Matrix6x J_cell;

      cc::VectorX a_zero_limits_;

      //////////////////////////////////////////////////////////////////////////
      // constraints
      //////////////////////////////////////////////////////////////////////////
      ConstraintInequality constraint_;

      //////////////////////////////////////////////////////////////////////////
      // catch infeasable conditions
      //////////////////////////////////////////////////////////////////////////      
      eiquadprog::solvers::EiquadprogFast qp_;
      Eigen::VectorXd a_;
      cc::MatrixX Aeq_;
      cc::MatrixX beq_;
      cc::MatrixX Q_;
      cc::VectorX p_;
      
      std::vector<double> mem_Aieq_;
      std::vector<double> mem_bieq_;

      std::vector<int> violated_starts_;
      std::vector<int> violated_sizes_;
      std::vector<int> violated_cells_;
      std::vector<int> patch_starts_;
      std::vector<int> patch_sizes_;

      Eigen::Matrix<double,4,3> G_;       // generator matrix
      Eigen::Matrix3d R_c_j_;             // contact wrt joint     
    };
    
  }
}

#endif