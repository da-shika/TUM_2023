#ifndef ICS_ROBOT_ROBOT_BASE_H_
#define ICS_ROBOT_ROBOT_BASE_H_

////////////////////////////////////////////////////////////////////////////////
// Using Pinocchio with Fast collision lib
////////////////////////////////////////////////////////////////////////////////
#define PINOCCHIO_WITH_HPP_FCL

////////////////////////////////////////////////////////////////////////////////
// pinocchio includes
////////////////////////////////////////////////////////////////////////////////
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/algorithm/geometry.hpp>

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/math/fwd.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/robot-wrapper.hpp>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/ros/parameters.h>
#include <control_core/interfaces/module_base.h>

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/interfaces/robot_wrapper_interface.h>
#include <ics_tsid_common/mobile_robot/mobile_robot_wrapper.h>
#include <ics_tsid_common/utilities/pinocchio.h>

////////////////////////////////////////////////////////////////////////////////
// skin includes
////////////////////////////////////////////////////////////////////////////////
#include <skin_model/skin_model.h>

////////////////////////////////////////////////////////////////////////////////
// ros includes
////////////////////////////////////////////////////////////////////////////////
#include <tf/transform_broadcaster.h>

namespace ics
{
  /**
   * @brief Robot Class holds the tsid::robots::RobotWrapper and 
   * adds new functionallity to this class.
   * Optionally loads the geometric model of the robot.
   * 
   */
  class RobotWrapper : 
    public cc::ModuleBase,
    public ics::RobotWrapperInterface
  {
  public:
    typedef cc::ModuleBase Base;
    typedef pinocchio::GeometryData GeometryData;
    typedef tsid::InverseDynamicsFormulationAccForce::Data Data;

  protected:
    bool verbose_;
    bool has_collision_model_;
    bool has_skin_model_;

    ////////////////////////////////////////////////////////////////////////////
    // pinocchio + tsid
    ////////////////////////////////////////////////////////////////////////////

    std::unique_ptr<tsid::robots::RobotWrapper> robot_;           // pinocchio robot wrapper
    std::unique_ptr<pinocchio::GeometryModel> geometry_model_;    // robot geometry model
    std::unique_ptr<skin::SkinModel> skin_model_;                 // skin model

    ////////////////////////////////////////////////////////////////////////////
    // members
    ////////////////////////////////////////////////////////////////////////////

    ics::BaseType base_type_;                             // robot base joint type
    cc::Scalar total_mass_;                               // robot mass
    std::vector<std::string> tsid_joint_names_;           // all joints, including floating base
    std::vector<std::string> virtual_frame_names_;        // virtual frame names
    std::vector<int> tsid_joint_ids_;                     // joint ids

    ////////////////////////////////////////////////////////////////////////////
    // ros connections
    ////////////////////////////////////////////////////////////////////////////
                           
    tf::TransformBroadcaster broadcaster_;                // broadcast newly added frames        
    std::vector<tf::StampedTransform> broadcaster_transformations_;

  public:
    RobotWrapper(const std::string& name);
    
    virtual ~RobotWrapper();

    /**
     * @brief integrate state (q,v) with acceleration a to new state (q_next, v_next)
     * 
     * @note This function also takes care of Lie algebra for floating base state
     * 
     * @param q 
     * @param v 
     * @param a 
     * @param dt 
     * @param q_next 
     * @param v_next 
     */
    void integrate(
      const cc::VectorX& q, const cc::VectorX& v, const cc::VectorX& a, 
      cc::Scalar dt, cc::VectorX& q_next, cc::VectorX& v_next);

    /**
     * @brief update kinematic 
     */
    void updateKinematic(const cc::VectorX& q, const cc::VectorX& v, Data& data);

    /**
     * @brief update geometric data
     * 
     * @param data 
     */
    void updateGeometry(const cc::VectorX& q, Data& data, GeometryData& geometry_data);

    /**
     * @brief robot base type
     * 
     * @return ics 
     */
    ics::BaseType baseType() const { return base_type_; }
    bool hasFloatingBase() const { return base_type_ == ics::FLOATING; }
    bool hasMobileBase() const { return base_type_ == ics::MOBILE; }
    bool hasFixedBase() const { return base_type_ == ics::FIXED; }

    /**
     * @brief has collision model loaded
     * 
     * @return true 
     * @return false 
     */
    bool hasCollisionModel() const { return has_collision_model_; }

    /**
     * @brief check if frame exists
     * 
     * @param name 
     * @return true 
     * @return false 
     */
    bool hasFrame(const std::string& name) const;

    /**
     * @brief get the frame id
     * 
     * @return pinocchio::FrameIndex 
     */
    pinocchio::FrameIndex frameId(const std::string& name) const;
    
    /**
     * @brief get the frame name
     * 
     * @param id 
     * @return std::string 
     */
    const std::string& frameName(const pinocchio::FrameIndex& id) const;
  
    /**
     * @brief broadcast frames to tf tree
     * 
     * @param time 
     */
    virtual void broadcastFrames(const ros::Time& time);


    /**
     * @brief return vector with joint indices
     * 
     * @return std::vector<int> 
     */
    std::vector<int> jointIndices() const;

    /**
     * @brief get the pose of a robot joint
     * 
     * @param data 
     * @param name 
     * @return pinocchio::SE3 
     */
    pinocchio::SE3 jointPosition(const Data& data, const std::string& name) const;


    /**
     * @brief get the pose of a robot frame wrt world frame
     * 
     * @param data 
     * @param name 
     * @return pinocchio::SE3 
     */
    pinocchio::SE3 framePosition(const Data& data, const pinocchio::FrameIndex& id) const;
    pinocchio::SE3 framePosition(const Data& data, const std::string& name) const;

    /**
     * @brief get relative transformation of child wrt parent frame
     * 
     * @param data 
     * @param parent 
     * @param child 
     * @return pinocchio::SE3 
     */
    pinocchio::SE3 relativeFramePosition(const Data& data, const std::string& parent, const std::string& child);

    /**
     * @brief get the velocity of a robot frame oriented in the world frame
     * 
     * @param data 
     * @param name 
     * @return pinocchio::SE3 
     */
    tsid::robots::RobotWrapper::Motion frameVelocity(const Data& data, const pinocchio::FrameIndex& id) const;
    tsid::robots::RobotWrapper::Motion frameVelocity(const Data& data, const std::string& name) const;

    /**
     * @brief get the cartesian state of a robot frame
     * 
     * @param datta 
     * @param name 
     * @return tsid::trajectories::TrajectorySample 
     */
    tsid::trajectories::TrajectorySample frameState(const Data& data, const pinocchio::FrameIndex& id) const;
    tsid::trajectories::TrajectorySample frameState(const Data& data, const std::string& name) const;

    /**
     * @brief get the name of the floatingbase joints
     * 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> fbJointNames() const;

    /**
     * @brief get the name of the actuated joints
     * 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> actuatedJointNames() const;

    /**
     * @brief get floating base and actuated joint names
     * 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> allJointNames() const;

    /**
     * @brief get vector of link masses
     * 
     * @return std::vector<cc::Scalar> 
     */
    std::vector<cc::Scalar> masses() const;
    
    /**
     * @brief get total robot mass
     * 
     * @return cc::Scalar 
     */
    cc::Scalar totalMass() const;

    ////////////////////////////////////////////////////////////////////////////
    // interface functions
    ////////////////////////////////////////////////////////////////////////////

    /**
     * @brief access to pinocchio robot wrapper
     * 
     */
    tsid::robots::RobotWrapper& wrapper() { return *robot_.get(); }
    const tsid::robots::RobotWrapper& wrapper() const { return *robot_.get(); }

    /**
     * @brief access to pinocchio robot model
     * 
     * @return pinocchio::Model& 
     */
    pinocchio::Model& model() { return robot_->model(); }
    const pinocchio::Model& model() const { return robot_->model(); }

    /**
     * @brief access to pinocchio geometry model
     * 
     * @return pinocchio::GeometryModel& 
     */
    pinocchio::GeometryModel& geometryModel() {
      if(!has_collision_model_)
        ROS_ERROR("Robot::geometryModel(): has no collision model loaded");
      return *geometry_model_.get(); 
    }
    const pinocchio::GeometryModel& geometryModel() const {
      if(!has_collision_model_)
        ROS_ERROR("Robot::geometryModel(): has no collision model loaded");
      return *geometry_model_.get(); 
    }

    /**
     * @brief print status
     */
    virtual std::string toString() const;

  protected:
    virtual bool init(ros::NodeHandle& nh, Base::Parameters& global_params) override;

  private:

    bool addVirtualFrames(cc::Parameters& parameter, const std::string& prefix, const std::string& tf_prefix);
  };
}

#endif