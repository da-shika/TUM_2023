#ifndef ICS_TSID_TASKS_invdyn_task_skin_compliance_hpp__
#define ICS_TSID_TASKS_invdyn_task_skin_compliance_hpp__

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
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

#include <ics_tsid_tasks/TaskSkinComplianceParameters.h>

namespace tsid
{
  namespace tasks
  {
    
    /**
     * @brief TaskSkinJointCompliance
     * 
     * Contact information through contact_generator message.
     * Transforms wrenches at the contact location to jointspace and then
     * sums contributions.
     * 
     * Action is computed as compliant torque and velocity damping.
     * 
     * Note: the dampening interfers with the posture task.
     * Better tune Kp and use dampening of posture task.
     */
    class TaskSkinJointCompliance : public TaskMotion
    {
    public:
      typedef TaskMotion Base;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::VectorXi VectorXi;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;
      typedef pinocchio::SE3 SE3;
      typedef pinocchio::Data::Matrix6x Matrix6x;

      typedef ics_tsid_tasks::TaskSkinComplianceConfig Config;
      typedef ics_tsid_tasks::TaskSkinComplianceServer Server;
      typedef ics_tsid_tasks::TaskSkinComplianceParameters Parameters;

      static std::shared_ptr<TaskBase> Load(
        ics::TSIDWrapperInterface& tsid_wrapper,
        ros::NodeHandle& nh,
        const std::string& name,
        bool activate = true,
        bool verbose = true);

    public:
      TaskSkinJointCompliance(const std::string & name,
                         RobotWrapper & robot,
                         const std::string& skinTopicName);

      int dim() const;

      bool subscribe(ros::NodeHandle& nh);

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      const Vector & getDesiredAcceleration() const;
      Vector getAcceleration(ConstRefVector dv) const;

      virtual void setMask(ConstRefVector mask);

      const Vector & Kp();
      const Vector & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

      const Vector & getTauExt() const;

      const double & getAlphaProximity() const;
      void setAlphaProximity(double alphaProximity);

    private:
      bool computeContactJacobian(const cc::SkinPatch& contact, Data & data, Matrix6x& Jc);

      void callback(const control_core_msgs::SkinPatchesConstPtr& msg);

      void reconfigureRequest(Config &config, uint32_t level);

    protected:
      bool has_patches_callback_;

      std::string skin_topic_name_;
      ros::Subscriber contacts_sub_;
      std::vector<cc::SkinPatch> contacts_;

      Parameters params_;
      std::unique_ptr<Server> reconfig_srv_;

      Vector Kp_;
      Vector Kd_;
      
      double alpha_proximity_;  // proximity weightening
      cc::Wrench W_des_;        // desired force on each contact

      Matrix6x J_c_;            // contact jacobian
      Vector tau_ext_;          // total external force

      SE3 T_f_w_, T_c_f_, T_c_w_;

      Vector v_;
      Vector a_des_;
      VectorXi active_axis_;
      ConstraintEquality constraint_;
    };
    
  }
}

#endif