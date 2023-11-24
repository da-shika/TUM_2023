#ifndef ICS_TSID_TASKS_invdyn_task_skin_distance_force_hpp__
#define ICS_TSID_TASKS_invdyn_task_skin_distance_force_hpp__

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
#include <ics_tsid_tasks/TaskSkinForceConstraintsParameters.h>
#include <ics_tsid_task_msgs/SkinForceConstraint.h>

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

namespace tsid
{
  namespace tasks
  {
    
    /**
     * @brief TaskSkinForceMotion a sub task used by TaskSkinForceConstraint
     * 
     * This task is added on top of TaskSkinForceConstraint to add an 
     * additional repulsive for component to the formulation.
     */
    class TaskSkinForceMotion : public TaskMotion
    {
      public:
        typedef TaskMotion Base;
      
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef math::Vector Vector;
        typedef math::VectorXi VectorXi;
        typedef math::ConstraintEquality ConstraintEquality;

      public:
        TaskSkinForceMotion(const std::string & name,
                              RobotWrapper & robot);
        
        int dim() const { return (int)m_mask.sum(); }

        virtual void setMask(math::ConstRefVector mask);

        virtual const ConstraintBase & compute(const double t,
                                      ConstRefVector q,
                                      ConstRefVector v,
                                      Data & data) override;

        const ConstraintBase & getConstraint() const { return constraint_; }
        ConstraintBase & getConstraint() { return constraint_; }

        const Vector& desAcceleration() const { return a_des_; }
        Vector& desAcceleration() { return a_des_; }

        const Vector& acceleration() const { return a_; }
        Vector& acceleration() { return a_; }

        const Vector& Kp() const { return Kp_; }
        void setKp(const Vector& Kp);

        const Vector& Kd() const { return Kd_; }
        void setKd(const Vector& Kd);

      private:
        Vector Kp_;
        Vector Kd_;

        Vector a_, a_des_;
        VectorXi active_axes_;
        ConstraintEquality constraint_;
    };
    
    /**
     * @brief TaskSkinForceConstraint
     * 
     * Reads the cell distances and adds them as 1 dim inequality contraints
     * to the formulation. 
     * 
     * @note This task should be added as a constraint, not a cost
     */
    class TaskSkinForceConstraint : public TaskMotion
    {
    public:
      typedef TaskMotion Base;
      typedef std::shared_ptr<TaskSkinForceMotion> MotionTask;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef math::Vector Vector;
      typedef math::ConstraintInequality ConstraintInequality;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;
      typedef pinocchio::Data::Matrix6 Matrix6;
      typedef pinocchio::Data::Matrix6x Matrix6x;

      typedef ics_tsid_tasks::TaskSkinForceConstraintsConfig Config;
      typedef ics_tsid_tasks::TaskSkinForceConstraintsServer Server;
      typedef ics_tsid_tasks::TaskSkinForceConstraintsParameters Parameters;

      typedef skin_client::SkinPatchClient::Ptr Client;
      typedef std::vector<Client> Clients;

      static std::shared_ptr<TaskBase> Load(
        ics::TSIDWrapperInterface& tsid_wrapper,
        ros::NodeHandle& nh,
        const std::string& name,
        bool activate = true,
        bool verbose = true);

    public:
      TaskSkinForceConstraint(const std::string & name,
                         cc::Scalar dt,
                         cc::Scalar max_weight,
                         RobotWrapper & robot, 
                         tsid::InverseDynamicsFormulationAccForce & formulation,
                         MotionTask motion_task);

      bool connect(ros::NodeHandle& nh);

      int dim() const { return (int)m_mask.sum(); }

      double maximumForce() const;

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

      tsid::InverseDynamicsFormulationAccForce & formulation_;

      //////////////////////////////////////////////////////////////////////////
      // settings
      //////////////////////////////////////////////////////////////////////////
      double dt_;
      double max_force_;
      double max_fmod_;
      double max_proximity_;
      double min_distance_;

      int num_active_cell_;
      int num_active_ieq_;
      int num_violated_ieq_;
      
      //////////////////////////////////////////////////////////////////////////
      // skin clients
      //////////////////////////////////////////////////////////////////////////
      Clients clients_;
      std::vector<size_t> m_client_ids;
      std::vector<size_t> client_start_idx_, client_end_idx_;

      std::vector<double> forces_lp_;
      
      //////////////////////////////////////////////////////////////////////////
      // transformations
      //////////////////////////////////////////////////////////////////////////
      pinocchio::SE3 X_c_b;
      pinocchio::SE3 placement;
      pinocchio::Motion v_joint, drift_b;
      Matrix6x J_cell;

      //////////////////////////////////////////////////////////////////////////
      // constraints
      //////////////////////////////////////////////////////////////////////////
      ConstraintInequality constraint_;

      //////////////////////////////////////////////////////////////////////////
      // motion task for relaxed violation
      //////////////////////////////////////////////////////////////////////////
      MotionTask motion_task_;
      double max_weight_;
      double weight_;
      double t_release_;

      //////////////////////////////////////////////////////////////////////////
      // debugging information
      //////////////////////////////////////////////////////////////////////////
      ros::Publisher msg_pub_;
      ics_tsid_task_msgs::SkinForceConstraint msg_;

    };
    
  }
}

#endif