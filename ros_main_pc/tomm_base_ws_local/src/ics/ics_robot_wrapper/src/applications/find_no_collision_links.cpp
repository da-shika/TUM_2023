#define PINOCCHIO_WITH_HPP_FCL

#include <tsid/robots/robot-wrapper.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include <control_core/ros/ros.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>

#include <geometry_msgs/PoseArray.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include <control_core/types.h>
#include <control_core/math.h>
#include <control_core/utilities/utilities.h>

////////////////////////////////////////////////////////////////////////////////
// pair of two bodies
////////////////////////////////////////////////////////////////////////////////
struct BodyPair
{
  std::string b1;
  std::string b2;

  BodyPair() {}
  BodyPair(const std::string& body1, const std::string& body2) : 
    b1(body1), b2(body2) {}

  bool same(const BodyPair& pair) const {
    return ((pair.b1 == b1 && pair.b2 == b2) || (pair.b1 == b2 && pair.b2 == b1));
  }
  
  std::string toString() const {
    std::stringstream ss;
    ss << "<disable_collisions link1=\"" << b1 
       << "\" link2=\"" << b2 
       << "\" reason=\"Never\" />";
    return ss.str();
  }
};

std::vector<BodyPair> 
remove(const std::vector<BodyPair>& all, const std::vector<BodyPair>& exclude)
{
  std::vector<BodyPair> remain = all;
  for(const auto& exclude_pair : exclude)
  {
    auto new_end = std::remove_if(remain.begin(), remain.end(), 
                    [exclude_pair](const BodyPair& element)
                    { return exclude_pair.same(element); });
    remain.erase(new_end, remain.end());
  }
  return remain;
}

/**
 * @brief Finds all links in urdf that do not have a collision (by sampling)
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_no_collision_bodies");
  ros::NodeHandle nh;
  
  //////////////////////////////////////////////////////////////////////////////
  // read inputs
  //////////////////////////////////////////////////////////////////////////////
  std::vector<std::string> inputs;
  if(argc > 1)
  {
    inputs.assign(argv + 1, argv + argc);
  }
  if(inputs.size() < 1)
  {
    ROS_ERROR("Missing Input arguments, Requires: [urdf_file_name, srdf_file_name, package_path]");
    return -1;
  }
  std::string urdf_file_name = inputs[0];
  std::string srdf_file_name = inputs[1];
  std::string package_path = inputs[2];

  ROS_INFO_STREAM("urdf_file_name: " << urdf_file_name);
  ROS_INFO_STREAM("srdf_file_name: " << srdf_file_name);
  ROS_INFO_STREAM("package_path: " << package_path);

  //////////////////////////////////////////////////////////////////////////////
  // build the robot model
  //////////////////////////////////////////////////////////////////////////////
  pinocchio::Model robot_model;
  pinocchio::urdf::buildModel(urdf_file_name, pinocchio::JointModelFreeFlyer(), robot_model, false);
  tsid::robots::RobotWrapper robot(robot_model, false);

  ROS_INFO_STREAM("--------- Robot size info ---------");
  ROS_INFO_STREAM("Number of actuators : " << robot.na());
  ROS_INFO_STREAM("Dimension of the configuration vector : " << robot.nq());
  ROS_INFO_STREAM("Dimension of the velocity vector : " << robot.nv());
  ROS_INFO_STREAM("---------------------------------");
  
  //////////////////////////////////////////////////////////////////////////////
  // build the geometry
  //////////////////////////////////////////////////////////////////////////////
  pinocchio::GeometryModel geometry_model;
  pinocchio::urdf::buildGeom(robot_model, urdf_file_name, pinocchio::COLLISION, geometry_model, package_path);
  geometry_model.addAllCollisionPairs(); 
  pinocchio::srdf::removeCollisionPairs(robot_model, geometry_model, srdf_file_name, true);

  pinocchio::Data data(robot_model);
  pinocchio::GeometryData geometry_data(geometry_model);

  cc::VectorX q_tsid = cc::VectorX::Zero(robot.nq());
  q_tsid.head(7) << 0, 0, 0, 0, 0, 0, 1;
  
  // add all collision pairs
  std::vector<BodyPair> all_collision_pairs;
  pinocchio::updateGeometryPlacements(
    robot_model, data, geometry_model, geometry_data, q_tsid);
  pinocchio::computeCollisions(geometry_model, geometry_data);
  for(size_t cp_index = 0; cp_index < geometry_model.collisionPairs.size(); ++cp_index)
  {
    const auto& cp = geometry_model.collisionPairs[cp_index];
    const pinocchio::GeometryObject& obj1 = geometry_model.geometryObjects[cp.first];
    const pinocchio::GeometryObject& obj2 = geometry_model.geometryObjects[cp.second];
    all_collision_pairs.push_back(BodyPair(obj1.name, obj2.name));
  }
  ROS_WARN("Total of %ld collision pairs", all_collision_pairs.size());

  //////////////////////////////////////////////////////////////////////////////
  // sampling
  //////////////////////////////////////////////////////////////////////////////

  // sample a bunch of jointstates and store all collision links
  std::random_device rd;
  std::mt19937 gen(rd());  //here you could also set a seed
  std::uniform_real_distribution<double> dis(0.0, 1.0);
  cc::VectorX q = cc::VectorX::Zero(robot.na());

  std::vector<BodyPair> collision_pairs;
  for(size_t k = 0; k < 10000; ++k)
  {
    // sample valid jointstate
    for(size_t i = 0; i < robot.na(); ++i)
    {
      cc::Scalar lo = robot.model().upperPositionLimit[i+7];
      cc::Scalar up = robot.model().lowerPositionLimit[i+7];
      q[i] = (up - lo)*dis(gen) + lo;
    }
    // set joint state for non base parts
    q_tsid.tail(robot.na()) = q;
    
    // update pose
    pinocchio::updateGeometryPlacements(
      robot_model, data, geometry_model, geometry_data, q_tsid);

    // compute collision
    pinocchio::computeCollisions(geometry_model, geometry_data);

    for(size_t cp_index = 0; cp_index < geometry_model.collisionPairs.size(); ++cp_index)
    {
      const auto& cp = geometry_model.collisionPairs[cp_index];
      const pinocchio::GeometryObject& obj1 = geometry_model.geometryObjects[cp.first];
      const pinocchio::GeometryObject& obj2 = geometry_model.geometryObjects[cp.second];
      const hpp::fcl::CollisionResult& res = geometry_data.collisionResults[cp_index];
      if(res.isCollision())
      {
        // there is a collision, this connection is important, remove it from the filter list
        collision_pairs.push_back(BodyPair(obj1.name, obj2.name));
      }
    }

    ROS_INFO_STREAM("SAMPLING STEP=" << k << " collision_pairs.size=" << collision_pairs.size());
  }

  //////////////////////////////////////////////////////////////////////////////
  // remove collision pairs form all_collision_pairs
  //////////////////////////////////////////////////////////////////////////////
  ROS_INFO_STREAM("REMOVING");
  // auto no_collision_paris = remove(all_collision_pairs, collision_pairs);
  auto no_collision_paris = all_collision_pairs;
  
  std::cout << "------------------------------------------------------------\n";
  for(const auto& pair : no_collision_paris)
  {
    std::cout << pair.toString() << std::endl;
  }
  std::cout << "------------------------------------------------------------\n";
  ROS_INFO_STREAM("all_collision_pairs size=" << all_collision_pairs.size()
   << " to no_collision_paris=" << no_collision_paris.size());

  return 0;
}