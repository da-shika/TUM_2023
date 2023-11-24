#ifndef ICS_TSID_UTILITES_CONVERSIONS_H_
#define ICS_TSID_UTILITES_CONVERSIONS_H_

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/trajectories/trajectory-base.hpp>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types.h>

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/utilities/pinocchio.h>

namespace ics
{
  /**
   * @brief TrajectorySample from Vector3
   * 
   * @param ref 
   * @return tsid::trajectories::TrajectorySample 
   */
  tsid::trajectories::TrajectorySample to_sample(const cc::VectorX& ref);

  /**
   * @brief TrajectorySample from CartesianPosition
   * 
   * @param ref 
   * @return tsid::trajectories::TrajectorySample 
   */
  tsid::trajectories::TrajectorySample to_sample(const cc::CartesianPosition& ref);

  /**
   * @brief TrajectorySample from SE3
   * 
   * @param ref 
   * @return tsid::trajectories::TrajectorySample 
   */
  tsid::trajectories::TrajectorySample to_sample(const pinocchio::SE3& ref);

  /**
   * @brief TrajectorySample from JointState
   * 
   * @param ref 
   * @return tsid::trajectories::TrajectorySample 
   */
  tsid::trajectories::TrajectorySample to_sample(const cc::JointState& ref);

  /**
   * @brief TrajectorySample from LinearState
   * 
   * @param ref 
   * @return tsid::trajectories::TrajectorySample 
   */
  tsid::trajectories::TrajectorySample to_sample(const cc::LinearState& ref);
  tsid::trajectories::TrajectorySample to_sample(const cc::AngularState& ref);
  
  /**
   * @brief TrajectorySample from CartesianState
   * 
   * @param ref 
   * @return tsid::trajectories::TrajectorySample 
   */
  tsid::trajectories::TrajectorySample to_sample(const cc::CartesianState& ref);

  /**
   * @brief trajectory sample position to se3 pose
   * 
   * @param ref 
   * @return pinocchio::SE3 
   */
  pinocchio::SE3 to_se3(const tsid::trajectories::TrajectorySample& ref);

  /**
   * @brief CartesianPosition to se3 pose
   * 
   * @param ref 
   * @return pinocchio::SE3 
   */
  pinocchio::SE3 to_se3(const cc::CartesianPosition& ref);

  /**
   * @brief convert se3 to cartesian pose
   * 
   * @param se3 
   * @return cc::CartesianPosition 
   */
  cc::CartesianPosition to_pose(const pinocchio::SE3& se3);
  cc::CartesianPosition to_pose(const tsid::trajectories::TrajectorySample& ref);
  cc::JointPosition to_joint_pos(const tsid::trajectories::TrajectorySample& ref);

  /**
   * @brief convert se3 to HomogeneousTransformation
   * 
   * @param se3 
   * @return cc::HomogeneousTransformation 
   */
  cc::HomogeneousTransformation to_transf(const pinocchio::SE3& se3);
  cc::CartesianPosition to_transf(const tsid::trajectories::TrajectorySample& ref);

  /**
   * @brief convert tsid sample to linear state
   * 
   * @param ref 
   * @return cc::LinearState 
   */
  cc::LinearState to_linear_state(const tsid::trajectories::TrajectorySample& ref);

  /**
   * @brief convert tsid sample to cartesian state
   * 
   * @param ref 
   * @return cc::CartesianState 
   */
  cc::CartesianState to_cartesian_state(const tsid::trajectories::TrajectorySample& ref);

  /**
   * @brief convert tsid sample to joint state
   * 
   * @param ref 
   * @return cc::JointState
   */
  cc::JointState to_joint_state(const tsid::trajectories::TrajectorySample& ref);

  /**
   * @brief convert sample to string for debugging
   * 
   * @param sample 
   * @return std::string 
   */
  std::string toString(const tsid::trajectories::TrajectorySample& sample);
  
  /**
   * @brief convert robotstate to configuration q and velocity v
   * 
   * @param robot_state 
   * @param q 
   * @param v 
   * @param base_type 
   */
  void to_tsid_state(const cc::RobotState& state, BaseType base_type, cc::VectorX& q, cc::VectorX& v);
  
  /**
   * @brief convert configuration q and velocity v to robotstate
   * 
   * @param q 
   * @param v 
   * @param robot_state 
   */
  void to_robot_state(const cc::VectorX& q, const cc::VectorX& v, BaseType base_type, cc::RobotState& state);

}

#endif
