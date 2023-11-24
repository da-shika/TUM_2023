#include <ics_robot_wrapper/robot_wrapper.h>

////////////////////////////////////////////////////////////////////////////////
// pinocchio includes
////////////////////////////////////////////////////////////////////////////////
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

////////////////////////////////////////////////////////////////////////////////
// urdf includes
////////////////////////////////////////////////////////////////////////////////
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

////////////////////////////////////////////////////////////////////////////////
// parameters
////////////////////////////////////////////////////////////////////////////////
#include <control_core/ros/parameters.h>

#include <ics_tsid_common/utilities/conversions.h>

namespace ics
{
  RobotWrapper::RobotWrapper(const std::string& name) : 
    Base(name),
    verbose_(false),
    robot_(nullptr),
    geometry_model_(nullptr)
  {
  }
  
  RobotWrapper::~RobotWrapper()
  {
  }

  bool RobotWrapper::init(ros::NodeHandle &nh, Base::Parameters& global_params)
  {

    ////////////////////////////////////////////////////////////////////////////
    // load parameters
    ////////////////////////////////////////////////////////////////////////////

    cc::Parameters robot_parameter(nh, Base::name());
    robot_parameter.addRequired<std::string>("base_type");                      // has_floating_base
    robot_parameter.addRequired<bool>("has_collision_model");                   // update the collsion model
    robot_parameter.addRequired<bool>("has_skin_model");                        // update the skin model
    robot_parameter.addOptional<std::string>("urdf_file", "");                  // path to urdf file
    robot_parameter.addOptional<std::string>("srdf_file", "");                  // path to srdf file
    robot_parameter.addOptional<std::string>("package_path", "");               // path to the robot descripition package
    robot_parameter.addOptional<std::string>("robot_description", "");          // or robot_description name on parameter server
    if(!robot_parameter.load())
    {
      PRINT_ERROR("Can't load Robot parameter");
      return false;
    }
    robot_parameter.get("has_collision_model", has_collision_model_);
    robot_parameter.get("has_skin_model", has_skin_model_);

    std::string base_type = robot_parameter.get<std::string>("base_type");
    if(base_type=="mobile")
      base_type_ = ics::BaseType::MOBILE;
    else if(base_type=="floating")
      base_type_ = ics::BaseType::FLOATING;
    else
      base_type_ = ics::BaseType::FIXED;

    ////////////////////////////////////////////////////////////////////////////
    // build urdf robot model
    ////////////////////////////////////////////////////////////////////////////

    std::string urdf_file = robot_parameter.get<std::string>("urdf_file");
    std::string robot_description_param = robot_parameter.get<std::string>("robot_description");
    pinocchio::Model robot_model;
    if(urdf_file.empty())
    {
      // instead load from ros parameter server
      if(robot_description_param.empty())
      {
        robot_description_param = "/robot_description";
      }
      std::string robot_description;
      if(!cc::load(robot_description_param, robot_description))
      {
        PRINT_ERROR("Can't read robot_description");
        return false;
      }

      // load urdf
      urdf::ModelInterfaceSharedPtr urdf_tree = urdf::parseURDF(robot_description);
      if(!urdf_tree)
      {
        PRINT_ERROR("Can't parse robot_description into urdf model");
        return false;
      }

      // create robot model
      if(!ics::build_model(urdf_tree, base_type_, robot_model, verbose_))
      {
        PRINT_ERROR("Can't build_model");
        return false;
      }
    }
    else
    {
      if(!ics::build_model(urdf_file, base_type_, robot_model, verbose_))
      {
        PRINT_ERROR("Can't build_model");
        return false;
      }
    }

    ////////////////////////////////////////////////////////////////////////////
    // create robot wrapper
    ////////////////////////////////////////////////////////////////////////////
    if(base_type_ == ics::MOBILE)
    {
      robot_ = std::make_unique<tsid::robots::MobileRobotWrapper>(robot_model, verbose_);
    }
    else
    {
      if(base_type_ == FLOATING)
        robot_ = std::make_unique<tsid::robots::RobotWrapper>(robot_model, verbose_);
      else
        robot_ = std::make_unique<tsid::robots::RobotWrapper>(robot_model, tsid::robots::RobotWrapper::RootJointType::FIXED_BASE_SYSTEM, verbose_);
    }

    ////////////////////////////////////////////////////////////////////////////
    // load and add additional frames to the model
    ////////////////////////////////////////////////////////////////////////////
    if(!addVirtualFrames(
      robot_parameter, global_params.get<std::string>("prefix"), global_params.get<std::string>("tf_prefix")))
    {
      PRINT_ERROR("Error adding virutal frames");
    }

    ////////////////////////////////////////////////////////////////////////////
    // build geometry model
    ////////////////////////////////////////////////////////////////////////////
    if(has_collision_model_)
    {
      std::string srdf_file = robot_parameter.get<std::string>("srdf_file");
      std::string package_path = robot_parameter.get<std::string>("package_path");

      if(urdf_file.empty() || package_path.empty())
      {
        PRINT_ERROR("has_collision_model but 'urdf_file' or 'package_path' or 'srdf_file' not set");
        has_collision_model_ = false;
        return false;
      }

      // create collision model
      geometry_model_ = std::make_unique<pinocchio::GeometryModel>();
      pinocchio::urdf::buildGeom(robot_model, urdf_file, pinocchio::COLLISION, *geometry_model_, package_path);
      geometry_model_->addAllCollisionPairs(); 

      if(!srdf_file.empty())
      {
        size_t prior_size = geometry_model_->collisionPairs.size();
        pinocchio::srdf::removeCollisionPairs(robot_model, *geometry_model_, srdf_file, verbose_);
        // if(verbose_)
        PRINT_WARN("Collision model: From '%s' mased out: %d bodypairs", 
          srdf_file.c_str(), int(prior_size - geometry_model_->collisionPairs.size()));
      }
      PRINT_WARN("Collision model: Total of %ld bodypairs found", geometry_model_->collisionPairs.size());
    }

    ////////////////////////////////////////////////////////////////////////////
    // build skin model
    ////////////////////////////////////////////////////////////////////////////
    if(has_skin_model_)
    {
      skin_model_ = std::make_unique<skin::SkinModel>("skin_model", robot_->model());
      if(!skin_model_->initRequest(nh, global_params))
      {
        PRINT_ERROR("failed to init skin model");
        return false;
      }
      cc::insert(skin_model_->surfaceFrameTfs(), broadcaster_transformations_);
    }

    ////////////////////////////////////////////////////////////////////////////
    // set members
    ////////////////////////////////////////////////////////////////////////////
    uint n_actuated = robot_->na();
    uint n_v = robot_->nv();
    uint n_q = robot_->nq();

    tsid_joint_names_ = allJointNames();
    tsid_joint_ids_ = jointIndices();
    total_mass_ = totalMass();

    ////////////////////////////////////////////////////////////////////////////
    // print status
    ////////////////////////////////////////////////////////////////////////////
    ROS_INFO_STREAM(toString());

    return true;
  }

  void RobotWrapper::integrate(
    const cc::VectorX& q, const cc::VectorX& v, const cc::VectorX& a, 
    cc::Scalar dt, cc::VectorX& q_next, cc::VectorX& v_next)
  {
    auto v_mean = v + 0.5 * dt * a;
    v_next = v + dt * a;
    q_next = pinocchio::integrate(robot_->model(), q, v_mean * dt);
  }

  void RobotWrapper::updateKinematic(const cc::VectorX& q, const cc::VectorX& v, Data& data)
  {
		pinocchio::forwardKinematics(robot_->model(), data, q, v);
		pinocchio::updateFramePlacements(robot_->model(), data);
  }

  void RobotWrapper::updateGeometry(const cc::VectorX& q, Data& data, GeometryData& geometry_data)
  {
    if(hasCollisionModel())
    {
      // update the collision checker
      pinocchio::updateGeometryPlacements(
        robot_->model(), data, *geometry_model_, geometry_data, q);

      // compute the collisions
      pinocchio::computeCollisions(*geometry_model_, geometry_data);
    }
  }

  std::vector<int> RobotWrapper::jointIndices() const
  { 
    std::vector<int> joint_ids(robot_->na());
    std::generate(joint_ids.begin(), joint_ids.end(), [n = 0]() mutable { 
      return n++; 
    });
    return joint_ids;
  }

  std::vector<std::string> RobotWrapper::fbJointNames() const
  {
    std::vector<std::string> floating_base_dofs;

    if (hasFloatingBase()) 
    {
      floating_base_dofs = {
        "rootJoint_pos_x",
        "rootJoint_pos_y",
        "rootJoint_pos_z",
        "rootJoint_rot_x",
        "rootJoint_rot_y",
        "rootJoint_rot_z"};
    }
    else if(hasMobileBase())
    {
      floating_base_dofs = {
        "rootJoint_pos_x",
        "rootJoint_pos_y",
        "rootJoint_rot_z"};
    }
    else 
    {
      floating_base_dofs = {};
    }
    return floating_base_dofs;
  }

  std::vector<std::string> RobotWrapper::actuatedJointNames() const
  {
    auto na = robot_->model().names;
    std::vector<std::string> tsid_controllables;
    if (hasFloatingBase() || hasMobileBase()) 
    {
      tsid_controllables = std::vector<std::string>(
        robot_->model().names.begin() + 2, robot_->model().names.end());
    }
    else 
    {
      tsid_controllables = std::vector<std::string>(
        robot_->model().names.begin() + 1, robot_->model().names.end());
    }
    return tsid_controllables;
  }

  std::vector<std::string> RobotWrapper::allJointNames() const
  {
    std::vector<std::string> all_dofs = fbJointNames();
    std::vector<std::string> actuated_dofs = actuatedJointNames();
    all_dofs.insert(all_dofs.end(), actuated_dofs.begin(), actuated_dofs.end());
    return all_dofs;
  }

  pinocchio::SE3 RobotWrapper::jointPosition(const Data& data, const std::string& name) const
  {
    if(!robot_->model().existJointName(name))
    {
      PRINT_ERROR("joint '%s' not exsisting", name.c_str());
      return pinocchio::SE3::Identity();
    }
    return robot_->position(data, robot_->model().getJointId(name));
  }

  pinocchio::SE3 RobotWrapper::framePosition(const Data& data, const std::string& name) const
  {
    if(!hasFrame(name))
    {
      PRINT_ERROR("frame '%s' not exsisting", name.c_str());
      return pinocchio::SE3::Identity();
    }
    return framePosition(data, frameId(name));
  }

  pinocchio::SE3 RobotWrapper::framePosition(const Data& data, const pinocchio::FrameIndex& id) const
  {
    return robot_->framePosition(data, id);
  }

  pinocchio::SE3 RobotWrapper::relativeFramePosition(const Data& data, const std::string& parent, const std::string& child)
  {
    return framePosition(data, parent).inverse()*framePosition(data, child);
  }

  tsid::robots::RobotWrapper::Motion RobotWrapper::frameVelocity(const Data& data, const std::string& name) const
  {
    if(!hasFrame(name))
    {
      PRINT_ERROR("frame '%s' not exsisting", name.c_str());
      return tsid::robots::RobotWrapper::Motion::Zero();
    }
    return frameVelocity(data, frameId(name));
  }

  tsid::robots::RobotWrapper::Motion RobotWrapper::frameVelocity(const Data& data, const pinocchio::FrameIndex& id) const
  {
    return robot_->frameVelocity(data, id);
  }

  tsid::trajectories::TrajectorySample RobotWrapper::frameState(const Data& data, const std::string& name) const
  {
    tsid::trajectories::TrajectorySample sample(12, 6);
    if(!hasFrame(name))
    {
      PRINT_ERROR("frame '%s' not exsisting", name.c_str());
    }
    return frameState(data, frameId(name));
  }

  tsid::trajectories::TrajectorySample RobotWrapper::frameState(const Data& data, const pinocchio::FrameIndex& id) const
  {
    tsid::trajectories::TrajectorySample sample(12, 6);
    tsid::math::SE3ToVector(robot_->framePosition(data, id), sample.pos);
    sample.setDerivative(robot_->frameVelocity(data, id).toVector());
    sample.setSecondDerivative(robot_->frameAcceleration(data, id).toVector());
    return sample;
  }

  std::vector<cc::Scalar> RobotWrapper::masses() const
  {
    std::vector<cc::Scalar> masses;
    for (int i = 0; i < robot_->model().inertias.size(); i++) 
    {
      masses.push_back(robot_->model().inertias[i].mass());
    }
    return masses;
  }

  cc::Scalar RobotWrapper::totalMass() const
  {
    double mass = 0.0;
    for (int i = 0; i < robot_->model().inertias.size(); i++) 
    {
      mass += robot_->model().inertias[i].mass();
    }
    return mass;
  }

  void RobotWrapper::broadcastFrames(const ros::Time& time)
  {
    for(size_t i = 0; i < broadcaster_transformations_.size(); ++i)
    {
      broadcaster_transformations_[i].stamp_ = time;
    }
    if(!broadcaster_transformations_.empty())
    {
      broadcaster_.sendTransform(broadcaster_transformations_);
    }
  }

  bool RobotWrapper::hasFrame(const std::string& name) const
  {
    return robot_->model().existFrame(name);
  }

  pinocchio::FrameIndex RobotWrapper::frameId(const std::string& name) const
  {
    return robot_->model().getFrameId(name);
  }

  const std::string& RobotWrapper::frameName(const pinocchio::FrameIndex& id) const
  {
    return robot_->model().frames[id].name;
  }

  bool RobotWrapper::addVirtualFrames(cc::Parameters& parameter, const std::string& prefix, const std::string& tf_prefix)
  {
    // get virtual frame names
    std::string frame_ns = "additional_frames";
    std::vector<std::string> child_names = cc::find_sub_names_spaces(frame_ns, prefix);

    // add to loader
    for(std::size_t i = 0; i < child_names.size(); ++i)
    {
      parameter.addRequired<std::string>(frame_ns + "/" + child_names[i] + "/parent");
      parameter.addRequired<cc::CartesianPosition>(frame_ns + "/" + child_names[i] + "/pose");
    }
    
    // load
    if(!parameter.load())
    {
      PRINT_ERROR("Error loading parameters");
      return false;
    }

    // add to model
    std::string parent_name;
    cc::CartesianPosition rel_pose;
    
    virtual_frame_names_.clear();
    broadcaster_transformations_.clear();
    for(std::size_t i = 0; i < child_names.size(); ++i)
    {
      const std::string& child_name = child_names[i];

      // extract parameter
      parameter.get(frame_ns + "/" + child_name + "/parent", parent_name);
      parameter.get(frame_ns + "/" + child_name + "/pose", rel_pose);

      // make sure quaternion is normalized
      rel_pose.angular().normalize();

      // create transformation relative to parent
      pinocchio::SE3 pose = pinocchio::SE3::Identity();
      pose.translation() = rel_pose.linear();
      pose.rotation() = rel_pose.angular().toRotationMatrix();

      if(!ics::add_frame_to_model(robot_->model(), parent_name, child_name, pose))
      {
        PRINT_ERROR("Error parent frame '%s' not exsisting", parent_name.c_str());
        return false;
      }

      // store virtual frame
      virtual_frame_names_.push_back(child_name);

      // create a new tf
      tf::Transform transform;
      transform.setIdentity();
      transform.setOrigin(rel_pose.linear());
      transform.setRotation(rel_pose.angular());
      broadcaster_transformations_.push_back(tf::StampedTransform(
        transform, ros::Time(0), tf_prefix + "/" + parent_name, tf_prefix + "/" + child_name));
    }
    return true;
  }

  std::string RobotWrapper::toString() const
  {
    std::stringstream ss;
    ss << "////////////////////////////////////////////////////////" << std::endl;
    ss << "//////////////////// Robot info ////////////////////////" << std::endl;
    ss << "Total Mass : " << total_mass_ << std::endl;
    ss << "Number of actuators : " << robot_->na() << std::endl;
    ss << "Dimension of the configuration vector : " << robot_->nq() << std::endl;
    ss << "Dimension of the velocity vector : " << robot_->nv() << std::endl;
    ss << "Joint Names:" << std::endl;
    for(auto& joint_name : tsid_joint_names_)
      ss << "- " << joint_name << std::endl;
    ss << "Virtual Model Frames:" << std::endl;
    for(auto& frame : virtual_frame_names_)
      ss << "- " << frame << std::endl;
    if(has_skin_model_) {
      ss << "Virtual Surface Frames:" << std::endl;
      for(auto& frame : skin_model_->surfaceFrameNames())
        ss << "- " << frame << std::endl;
    }
    ss << "////////////////////////////////////////////////////////" << std::endl;

    return ss.str();
  }

}
