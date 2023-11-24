#ifndef ICS_TSID_UTILITES_PINOCCHIO_H_
#define ICS_TSID_UTILITES_PINOCCHIO_H_

////////////////////////////////////////////////////////////////////////////////
// pinocchio includes
////////////////////////////////////////////////////////////////////////////////
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

////////////////////////////////////////////////////////////////////////////////
// urdf includes
////////////////////////////////////////////////////////////////////////////////
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

////////////////////////////////////////////////////////////////////////////////
// ros includes
////////////////////////////////////////////////////////////////////////////////
#include <ros/console.h>

namespace ics
{
  /**
   * @brief The Base JointType used by this robot
   */
  enum BaseType { FIXED, MOBILE, FLOATING };

  /**
   * @brief build the pinocchio model from given urdf tree
   * 
   * @param urdf_tree 
   * @param base_type 
   * @param robot_model 
   * @param verbose 
   * @return true 
   * @return false 
   */
  inline bool build_model(urdf::ModelInterfaceSharedPtr urdf_tree, BaseType base_type, pinocchio::Model& robot_model, bool verbose=true)
  {
    if(base_type == FLOATING)
    {
      pinocchio::urdf::buildModel(urdf_tree, pinocchio::JointModelFreeFlyer(), robot_model, verbose);
      if(verbose)
        ROS_WARN("build_model(): Adding floating base joint to the loaded urdf model");
    }
    else if(base_type == MOBILE)
    {
      pinocchio::urdf::buildModel(urdf_tree, pinocchio::JointModelPlanar(), robot_model, verbose);
      if(verbose)
        ROS_WARN("build_model(): Adding mobile base joint to the loaded urdf model");
    }
    else if(base_type == FIXED)
    {
      pinocchio::urdf::buildModel(urdf_tree, robot_model, verbose);
      if(verbose)
        ROS_WARN("build_model(): Adding no base joint");
    }
    else
    {
      ROS_ERROR("build_model(): Unkown base type");
      return false;
    }
    return true;
  }

  /**
   * @brief build the pinocchio model from given urdf file
   * 
   * @param urdf_file 
   * @param base_type 
   * @param robot_model 
   * @param verbose 
   * @return true 
   * @return false 
   */
  inline bool build_model(const std::string& urdf_file, BaseType base_type, pinocchio::Model& robot_model, bool verbose=true)
  {
    if(base_type == FLOATING)
    {
      // no floating base in the model, add FreeFlyer Joint here
      pinocchio::urdf::buildModel(urdf_file, pinocchio::JointModelFreeFlyer(), robot_model, verbose);
      if(verbose)
        ROS_WARN("build_model(): Adding floating base joint to the loaded urdf model");
    }
    else if(base_type == MOBILE)
    {
      pinocchio::urdf::buildModel(urdf_file, pinocchio::JointModelPlanar(), robot_model, verbose);
      if(verbose)
        ROS_WARN("build_model(): Adding mobile base joint to the loaded urdf model");
    }
    else if(base_type == FIXED)
    {
      pinocchio::urdf::buildModel(urdf_file, robot_model, verbose);
      if(verbose)
        ROS_WARN("build_model(): Adding no base joint");
    }
    else
    {
      ROS_ERROR("build_model(): Unkown base type");
      return false;
    }
    return true;
  }
  
  /**
   * @brief add a frame to the model
   * 
   * @param model 
   * @param parent 
   * @param child 
   * @param pose 
   * @return true 
   * @return false 
   */
  inline bool add_frame_to_model(pinocchio::Model& model, const std::string& parent, const std::string& child, const pinocchio::SE3& pose)
  {
    // get parent frame
    if(!model.existFrame(parent))
    {
      return false;
    }
    auto parent_frame_id = model.getFrameId(parent);
    auto& frame = model.frames[parent_frame_id];

    // add child frame
    model.addFrame(pinocchio::Frame(
      child, frame.parent, parent_frame_id, 
      frame.placement*pose, pinocchio::FIXED_JOINT));
    return true;
  }

}

#endif