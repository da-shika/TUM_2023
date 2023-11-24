#ifndef ICS_TSID_TASKS_invdyn_task_self_collision_constraint_hpp__
#define ICS_TSID_TASKS_invdyn_task_self_collision_constraint_hpp__

////////////////////////////////////////////////////////////////////////////////
// Using Pinocchio with Fast collision lib
////////////////////////////////////////////////////////////////////////////////
#define PINOCCHIO_WITH_HPP_FCL

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/tasks/task-motion.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/math/constraint-bound.hpp>
#include <tsid/math/constraint-inequality.hpp>

////////////////////////////////////////////////////////////////////////////////
// pinocchio includes
////////////////////////////////////////////////////////////////////////////////
#include <pinocchio/algorithm/geometry.hpp>

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/interfaces/tsid_wrapper_interface.h>
#include <ics_tsid_task_msgs/SelfCollisionConstraint.h>

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
     * @brief TaskSelfCollisionConstraint
     * 
     * Compute the self collision between body pairs.
     * Number of body pairs is set as fixed and inserted as 
     * a acceleration constrain.
     * 
     */
    class TaskSelfCollisionConstraint : public TaskMotion
    {
    public:
      typedef TaskMotion Base;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Vector3 Vector3;
      typedef math::Vector6 Vector6;
      typedef math::VectorXi VectorXi;
      typedef math::ConstraintInequality ConstraintInequality;
      typedef pinocchio::Data Data;
      typedef pinocchio::Data::Matrix6x Matrix6x;
      typedef pinocchio::Data::Matrix6 Matrix6;
      typedef pinocchio::GeometryModel GeometryModel;
      typedef pinocchio::GeometryData GeometryData;
      typedef Eigen::Matrix<double, 1, Eigen::Dynamic> Matrix1x;

      static std::shared_ptr<TaskBase> Load(
        ics::TSIDWrapperInterface& tsid_wrapper,
        ros::NodeHandle& nh,
        const std::string& name,
        bool activate = true,
        bool verbose = true);

    public:
      TaskSelfCollisionConstraint(const std::string & name,
                                  cc::Scalar dt,
                                  cc::Scalar v_damp,
                                  cc::Scalar d_min,
                                  cc::Scalar d_start,
                                  GeometryModel & geometry_model,
                                  GeometryData & geometry_data,
                                  RobotWrapper & robot);

      bool connect(ros::NodeHandle& nh);

      int dim() const;

      double minimumDistance() const;

      int numActiveConstraints() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      virtual void setMask(math::ConstRefVector mask);

    private:
      void visualize();

    protected:
      double m_dt;
      double m_v_max;
      double m_d_min;
      double m_d_start;
      double m_eps;

      double min_distance_;
      int num_active_ieq_;
      int num_violated_ieq_;
      
      pinocchio::GeometryModel& m_geometry_model;
      pinocchio::GeometryData& m_geometry_data;

      // collision space mappings
      Matrix6x J_c1, J_c2;
      Matrix1x J_12;
      Vector3 n_12;
      
      // transformations
      pinocchio::SE3 X_1_w, X_2_w;      // transformations collision point to world
      pinocchio::SE3 R_1_w, R_2_w;      // rotations collision point to world
      pinocchio::SE3 X_c1_b, X_c2_b;    // collision point wrt joint

      // acceleration drifts
      Vector6 drift1, drift2;           // drifts world oriented

      // temporaries
      pinocchio::SE3 placement;
      pinocchio::Motion drift_1_b, drift_2_b; // drifts in local body frame
      pinocchio::Motion v_joint;

      // visualization
      double t_prev_vis;
      ros::Publisher distance_pt_pub;
      ros::Publisher distance_n_pub;
      visualization_msgs::Marker distance_pt_marker;
      visualization_msgs::Marker distance_n_marker;

      Vector m_v;
      Vector m_a_des;
      VectorXi m_activeAxes;
      ConstraintInequality m_constraint;

      //////////////////////////////////////////////////////////////////////////
      // debugging information
      //////////////////////////////////////////////////////////////////////////
      ros::Publisher msg_pub_;
      ics_tsid_task_msgs::SelfCollisionConstraint msg_;
    };
    
  }
}

#endif // ifndef __invdyn_task_self_collision_constraint_hpp__