
#ifndef ICS_TSID_TASKS_invdyn_task_skin_wrench_hpp__
#define ICS_TSID_TASKS_invdyn_task_skin_wrench_hpp__

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"

////////////////////////////////////////////////////////////////////////////////
// pinocchio includes
////////////////////////////////////////////////////////////////////////////////
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

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

////////////////////////////////////////////////////////////////////////////////
// skin_client includes
////////////////////////////////////////////////////////////////////////////////
#include <skin_client/patch_client_factory.h>

namespace tsid
{
  namespace tasks
  {
    
    /**
     * @brief TaskSkinContactSoft
     * 
     * Force tracking task for soft skin contacts. 
     * Soft since contact is modled as a motion tasks with real 
     * skin force feedback (Here we don't add constraints to the problem)
     * 
     * Optionally: Add motion references to move the contact frame (set Kp gain > 0)
     * Note: This conflicts the force task.
     * 
     * Actions:
     * - Normal Action is computed as PI force controller with velocity damping.
     * - Angular torque Action is computed as P torque controller with velocity damping.
     * - Motion Action task is PD controller
     * 
     * Reference frame:
     * - Mod 1: Reference force expressed in contact frame, Motion reference expressed in motion frame
     * - Mod 2: Reference force expressed in contact frame, Motion reference expressed in contact frame
     * Mod 1 is active if a motion_frame is not empty (we got a motion frame)
     * 
     * Default Dimension = 3 with fz, tau_x, tau_y
     * 
     */
    class TaskSkinContactSoft : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Vector6 Vector6;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;
      typedef pinocchio::Data::Matrix6x Matrix6x;
      typedef pinocchio::Motion Motion;
      typedef pinocchio::SE3 SE3;
      typedef pinocchio::Frame Frame;

      typedef skin_client::SkinPatchClient::Ptr Client;
      
      enum RefFrame { REF_IN_CONTACT_FRAME, REF_IN_MOTION_FRAME };

      /**
       * @brief Construct a new TaskSkinContactSoft object
       */
      static std::shared_ptr<TaskSkinContactSoft> Make(
        ics::TSIDWrapperInterface& tsid_wrapper,
        const std::string& name,
        cc::Scalar dt,
        const std::string & contact_topic,
        const std::string & patch_frame,
        const std::string & motion_frame,
        cc::Scalar weight,
        ConstRefVector kf, 
        ConstRefVector kp,
        ConstRefVector kd,
        ConstRefVector ki,
        math::ConstRefVector mask,
        ros::NodeHandle& nh,
        bool activate = true,
        bool verbose = true);

      /**
       * @brief Load a new TaskSkinContactSoft object from ros parameters
       */
      static std::shared_ptr<TaskBase> Load(
        ics::TSIDWrapperInterface& tsid_wrapper,
        ros::NodeHandle& nh,
        const std::string& name,
        bool active = true,
        bool verbose = true);

    public:
      TaskSkinContactSoft(const std::string & name,
                            RobotWrapper & robot,
                            const double dt,
                            const std::string & contact_topic,
                            const std::string & patch_frame,
                            const std::string & motion_frame,
                            math::ConstRefVector mask = Vector::Ones(6));

      bool connect(ros::NodeHandle& nh);

      int dim() const { return (int)m_mask.sum(); }

      bool hasConnection() const { return has_contact_; }

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      /**
       * @brief Set the force reference
       */
      void setForceReference(const Vector6 & ref);

      /**
       * @brief Set the Motion Reference
       */
      void setMotionReference(const TrajectorySample & ref);

      const TrajectorySample & getMotionReference() const { return motion_ref_; } 
      const Vector6 & getForceReference() const { return force_ref_; }
      const Vector6 & getExternalForce() const { return fext_; }

      void setExternalForce(const Vector6 & fext) { 
        contact_.force().W() = fext; }

      const cc::SkinPatch & getContact() const { return contact_; }
      const Vector6 & getSkinWrench() const { return skin_wrench_; }
      const Vector6 & getSkinProximity() const { return skin_prox_; }

      virtual void setMask(math::ConstRefVector mask);

      /** Return the desired task acceleration (after applying the specified mask).
       */
      const Vector & getDesiredAcceleration() const;

      /** Return the task acceleration (after applying the specified mask).
       */
      Vector getAcceleration(ConstRefVector dv) const;

      /** Return the task velocity.
       */
      const Vector & velocity() const;                                          

      const Vector & Kf() const { return Kf_; }
      const Vector & Kp() const { return Kp_; }
      const Vector & Kd() const { return Kd_; }
      const Vector & Ki() const { return Ki_; }
      const double & getLeakRate() const { return leak_rate_; }
      const double & getProximityWeight() const { return proximity_weight_; }

      void Kf(ConstRefVector Kf);
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);
      void Ki(ConstRefVector Ki);
      void setLeakRate(double leak);
      void setProximityWeight(double proximity_weight);

      Index frame_id() const;

    private:
      void callback(const control_core_msgs::SkinPatchesConstPtr& msg);

    protected:
      //////////////////////////////////////////////////////////////////////////
      // utilities
      //////////////////////////////////////////////////////////////////////////
      RefFrame ref_frame_;                  // reference frame mode
      bool has_contact_;                    // contact topic recieved

      std::string contact_topic_;
      std::string motion_frame_;
      std::string patch_frame_;
      Index frame_id_, joint_id_;
      Frame frame_;

      double proximity_weight_;             // mixing of proximity and froce
      double leak_rate_;                    // integrator time constanct
      double dt_;  

      //////////////////////////////////////////////////////////////////////////
      // contact feeback
      //////////////////////////////////////////////////////////////////////////
      ros::Subscriber wbc_contacts_sub_;    // current skin contacts feedback 
      cc::SkinPatch contact_;               // contact container
      Vector6 skin_prox_;                   // skin proximity in motion frame
      Vector6 skin_wrench_;                 // skin wrench in motion frame

      //////////////////////////////////////////////////////////////////////////
      // gains
      //////////////////////////////////////////////////////////////////////////
      Vector Kf_;
      Vector Kp_;
      Vector Kd_;
      Vector Ki_;  
      
      //////////////////////////////////////////////////////////////////////////
      // force task
      //////////////////////////////////////////////////////////////////////////
      Vector6 force_ref_;                       // reference Force 6D to follow (in motion frame)
      Vector6 fext_;                            // external Force 6D in motion frame

      Vector forceIntegralError_;               // Integral error of the PID

      //////////////////////////////////////////////////////////////////////////
      // motion task
      //////////////////////////////////////////////////////////////////////////
      TrajectorySample motion_ref_;             // reference Motion 6D to follow
      SE3 T_ref_;                               // motion ref
      SE3 T_m_w_, R_m_w_;                       // motion frame pose and rotation
      SE3 T_c_f_, T_c_j_;                         // contact wrt frame and joint 
      cc::CartesianPosition X_p_m_;             // patch frame to motion frame
      cc::CartesianPosition X_c_m_;             // contact to motion frame
      cc::Matrix6 Ad_c_m_;
      
      Motion v_ref_, a_ref_;
      Vector p_error_vec_, v_error_vec_, f_error_vec_;
      Motion p_error_, v_error_;                // motion error signals

      //////////////////////////////////////////////////////////////////////////
      // robot motion
      //////////////////////////////////////////////////////////////////////////
      Vector a_des_, a_des_masked_, drift_masked_;
      Motion drift_, v_;
      Vector v_vec_;
      Matrix6x J_;
      ConstraintEquality constraint_;
    };

  }
}

#endif // ifndef __invdyn_task_se3_equality_hpp__
