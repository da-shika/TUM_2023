#ifndef ICS_TSID_TASKS_invdyn_task_contact_distance_constraint_soft_hpp__
#define ICS_TSID_TASKS_invdyn_task_contact_distance_constraint_soft_hpp__

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

namespace tsid
{
  namespace tasks
  {
    
    /**
     * @brief TaskContactDistanceConstraintMotion a sub task used by TaskContactDistanceConstraint
     * 
     * This task is added on top of TaskContactDistanceConstraint to add an 
     * additional repulsive for component to the formulation.
     */
    class TaskContactDistanceConstraintMotion : public TaskMotion
    {
      public:
        typedef TaskMotion Base;
      
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef math::Vector Vector;
        typedef math::VectorXi VectorXi;
        typedef math::ConstraintEquality ConstraintEquality;

      public:
        TaskContactDistanceConstraintMotion(const std::string & name,
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
     * @brief TaskContactDistanceConstraint
     * 
     * Reads control_core_msgs::SkinPatches from contact generator topic
     * and creates constraints based on distance measurment.
     * 
     * Unfeasible constraints relaxed into the cost function by turning them
     * into a potential field handled by seperate motion task
     * (I call this soft)
     * 
     * @note This task should be added as a constraint, not a cost
     */
    class TaskContactDistanceConstraintSoft : public TaskMotion
    {
    public:
      typedef TaskMotion Base;
      typedef std::shared_ptr<TaskContactDistanceConstraintMotion> MotionTask;

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
      TaskContactDistanceConstraintSoft(const std::string & name,
                         cc::Scalar dt,
                         cc::Scalar max_weight,
                         const std::string & contact_topic,
                         RobotWrapper & robot,
                         tsid::InverseDynamicsFormulationAccForce & formulation,
                         MotionTask motion_task);

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
      
      pinocchio::SE3 T_c_f_, T_c_j_, T_c_w_, T_f_w_;
      pinocchio::Motion v_joint_, drift_;
      Matrix6x J_;
      cc::Vector3 n_;

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
      ics_tsid_task_msgs::SkinDistanceConstraint msg_;

    };
    
  }
}

#endif