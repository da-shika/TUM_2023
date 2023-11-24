#ifndef ICS_TSID_TASKS_invdyn_task_contact_distance_constraint_hard_hpp__
#define ICS_TSID_TASKS_invdyn_task_contact_distance_constraint_hard_hpp__

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
#include <ics_tsid_task_msgs/SkinDistanceConstraint.h>

////////////////////////////////////////////////////////////////////////////////
// ros includes
////////////////////////////////////////////////////////////////////////////////
#include <unordered_map>
#include <ros/ros.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types.h>
#include <control_core_msgs/SkinPatches.h>

////////////////////////////////////////////////////////////////////////////////
// qp solver includes
////////////////////////////////////////////////////////////////////////////////
#include <eiquadprog/eiquadprog-fast.hpp>

namespace tsid
{
  namespace tasks
  {
    
    /**
     * @brief TaskContactDistanceConstraint (Test)
     * 
     * Reads control_core_msgs::SkinPatches from contact generator topic
     * and creates constraints based on distance measurment.
     * 
     * Unfeasible constraints are filtered first through another qp.
     * This qp checks if the repulsive action of violated constraints
     * leads to infeasibilites and removes them before handing them over to 
     * tsid's wbqp (I call this method "hard").
     * 
     * @note This task should be added as a constraint, not a cost
     */
    class TaskContactDistanceConstraintHard : public TaskMotion
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

      static std::shared_ptr<TaskBase> Load(
        ics::TSIDWrapperInterface& tsid_wrapper,
        ros::NodeHandle& nh,
        const std::string& name,
        bool activate = true,
        bool verbose = true);

    public:
      TaskContactDistanceConstraintHard(const std::string & name,
                         cc::Scalar dt,
                         const std::string & contact_topic,
                         RobotWrapper & robot,
                         tsid::InverseDynamicsFormulationAccForce & formulation);

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
      void callback(const control_core_msgs::SkinPatchesConstPtr& msg);

    private:
        void reconfigureRequest(Config &config, uint32_t level);

    protected:
      Parameters params_;
      std::unique_ptr<Server> reconfig_srv_;

      tsid::InverseDynamicsFormulationAccForce & formulation_;

      //////////////////////////////////////////////////////////////////////////
      // settings
      //////////////////////////////////////////////////////////////////////////
      double dt_;
      double min_distance_;
      double max_force_;
      double max_proximity_;
      int num_active_ieq_;
      int num_violated_ieq_;
      size_t num_max_ieq_;
      
      //////////////////////////////////////////////////////////////////////////
      // contact feeback
      //////////////////////////////////////////////////////////////////////////
      std::string contact_topic_;
      ros::Subscriber wbc_contacts_sub_;    // current skin contacts feedback 
      cc::SkinPatches contacts_;            // contact containers

      //////////////////////////////////////////////////////////////////////////
      // transformations
      //////////////////////////////////////////////////////////////////////////
      std::unordered_map<std::string, Index> frame_ids_;
      
      pinocchio::SE3 T_f_w_, T_c_w_, T_c_f_, T_c_j_;
      pinocchio::Motion v_joint_, drift_;
      Matrix6x J_;
      cc::Vector3 n_;

      cc::VectorX a_zero_limits_;

      //////////////////////////////////////////////////////////////////////////
      // constraints
      //////////////////////////////////////////////////////////////////////////
      ConstraintInequality constraint_;

      //////////////////////////////////////////////////////////////////////////
      // catch infeasable conditions
      //////////////////////////////////////////////////////////////////////////      
      eiquadprog::solvers::EiquadprogFast qp_;
      cc::MatrixX Aeq_;
      cc::MatrixX beq_;
      cc::MatrixX Q_;
      cc::VectorX p_;

      cc::VectorX sol_prime_;
      cc::MatrixX Q_prime_;
      cc::VectorX p_prime_;
      cc::MatrixX Aieq_prime_;
      cc::VectorX bieq_prime_;
      
      std::vector<double> mem_Aieq_;
      std::vector<double> mem_bieq_;

      std::vector<int> active_frame_ids_;
      std::vector<int> violated_frame_ids_;
      std::vector<int> violated_indices_;

      Eigen::Matrix<double,4,3> G_;       // generator matrix
      Eigen::Matrix3d R_c_j_;             // contact wrt joint     

      //////////////////////////////////////////////////////////////////////////
      // debugging information
      //////////////////////////////////////////////////////////////////////////
      ros::Publisher msg_pub_;
      ics_tsid_task_msgs::SkinDistanceConstraint msg_;

    };
    
  }
}

#endif