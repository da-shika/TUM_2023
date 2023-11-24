/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <boost/algorithm/string.hpp>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/MultiRayShape.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Collision.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/Noise.hh"

#include <gazebo_skin_plugin/skin_sensor.h>

#include <pcl/cloud_iterator.h>
#include <pcl_conversions/pcl_conversions.h>

#include <math.h>

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("skin", SkinSensor)

//////////////////////////////////////////////////
SkinSensor::SkinSensor() : 
  Sensor(sensors::RAY),
  is_updated_(false),
  is_first_(true)
{
}

//////////////////////////////////////////////////
SkinSensor::~SkinSensor()
{
}

//////////////////////////////////////////////////
std::string SkinSensor::Topic() const
{
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/scan";
  boost::replace_all(topicName, "::", "/");
  return topicName;
}

//////////////////////////////////////////////////
void SkinSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  GZ_ASSERT(this->world != nullptr,
      "SkinSensor did not get a valid World pointer");

  physics::PhysicsEnginePtr physicsEngine = this->world->Physics();

  GZ_ASSERT(physicsEngine != nullptr,
      "Unable to get a pointer to the physics engine");

  laser_collision_ = physicsEngine->CreateCollision("multiray",
      this->ParentName());

  GZ_ASSERT(laser_collision_ != nullptr,
      "Unable to create a multiray collision using the physics engine.");

  laser_collision_->SetName("ray_sensor_collision");
  laser_collision_->SetRelativePose(this->pose);
  laser_collision_->SetInitialRelativePose(this->pose);

  laser_shape_ =
    boost::dynamic_pointer_cast<physics::MultiRayShape>(
        laser_collision_->GetShape());

  GZ_ASSERT(laser_shape_ != nullptr,
      "Unable to get the laser shape from the multi-ray collision.");

  laser_shape_->Load(this->sdf);

  sdf::ElementPtr patch = this->sdf->GetElement("ray");
  if (patch->HasElement("noise"))
  {
    this->noises[RAY_NOISE] =
        NoiseFactory::NewNoiseModel(patch->GetElement("noise"),
        this->Type());
  }
  
  // sensor signals wrt parent frame
  parent_entity_ = this->world->EntityByName(this->ParentName());
  GZ_ASSERT(parent_entity_ != nullptr, "Unable to get the parent entity.");
  
  if(parent_entity_->GetChildCount() > 0)
  {
    parent_collision_ = boost::dynamic_pointer_cast<gazebo::physics::Collision>(parent_entity_->GetChild(0));
  }

  for(size_t i = 0; i < parent_entity_->GetChildCount(); ++i)
  {
    gazebo::physics::CollisionPtr geom = boost::dynamic_pointer_cast<gazebo::physics::Collision>(parent_entity_->GetChild(i));
    if(geom)
    {
      printf("Parent: %s has collision %s\n", 
        parent_entity_->GetName().c_str(), geom->GetName().c_str());
    }
  }

  // get the desired frame of the sensor data 
  sdf::ElementPtr plugin = this->sdf->GetElement("plugin");
  header_.frame_id = plugin->Get<std::string>("frameName");

  // missuse the topic name as xml name
  patch_xml_name_ = plugin->Get<std::string>("topicName");

  // init clouds
  proximity_cloud_.reset(new PointCloud());
  cell_cloud_.reset(new PointXYZCloud());
  normal_cloud_.reset(new PointXYZCloud());
  cell_parent_cloud_.reset(new PointXYZCloud());
  normal_parent_cloud_.reset(new PointXYZCloud());
}

//////////////////////////////////////////////////
void SkinSensor::Init()
{
  // load the xml file
  std::string xml_file;

  bool ok = ros::param::get(patch_xml_name_, xml_file);
  GZ_ASSERT(ok != false, "SkinSensor::Init(): Unable get patch_xml content form parameter server");

  ok = intializeFromXML(xml_file);
  GZ_ASSERT(ok != false, "SkinSensor::Init(): Unable to initalize skin_sensor from xml_file");

  // setup the publisher for pointclouds and server
  nh_ = ros::NodeHandle("");
  server_.enableDataConnection(nh_);
  
  // init base
  Sensor::Init();
}

//////////////////////////////////////////////////
void SkinSensor::Fini()
{
  Sensor::Fini();

  if (laser_collision_)
  {
    laser_collision_->Fini();
    laser_collision_.reset();
  }

  if (laser_shape_)
  {
    laser_shape_->Fini();
    laser_shape_.reset();
  }
}

SkinSensor::PointCloud::ConstPtr SkinSensor::proximityCloud()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return proximity_cloud_;
}

SkinSensor::PointXYZCloud::ConstPtr SkinSensor::cellCloud() const
{
  return cell_cloud_;
}

SkinSensor::PointXYZCloud::ConstPtr SkinSensor::normalCloud() const
{
  return normal_cloud_;
}

const skin_client::SkinPatchServer& SkinSensor::patchServer() const
{
  return server_;
}

//////////////////////////////////////////////////
bool SkinSensor::UpdateImpl(const bool /*_force*/)
{
  const float invalid = std::numeric_limits<float>::quiet_NaN();

  // set the collision mask
  if(is_first_)
  {
    //is_first_ = false;
    setCollisionMasks();
  }

  // only need to run collision checks if someone is listening to this patch
  if(server_.numberOfSubscriber() == 0)
  {
    return true;
  }

  // do the collision checks
  // this eventually call OnNewScans, so move mutex lock behind it in case
  // need to move mutex lock after this? or make the OnNewLaserScan connection
  // call somewhere else?
  laser_shape_->Update();
  this->lastMeasurementTime = this->world->SimTime();

  // moving this behind laserShape update
  std::lock_guard<std::mutex> lock(mutex_);

  // update server
  if(server_.numberOfSubscriber() > 0)
  {
    std::vector<double>& prox = server_.dynamicData().prox;
    std::vector<double>& dist = server_.dynamicData().dist;
    for(size_t i = 0; i < prox.size(); ++i)
    {
      double range, intensity;
      range = laser_shape_->GetRange(i);
      intensity = laser_shape_->GetRetro(i);

      if(range >= min_range_ && range <= max_range_) {
        prox[i] = distanceToProximity(range);  
        dist[i] = range;
      }
      else {
        prox[i] = 0.0;
        dist[i] = max_range_;
      }

      // send to ros
      server_.publish();
    }
  }
  return true;
}

bool SkinSensor::IsActive() const
{
  return true;
}

physics::MultiRayShapePtr SkinSensor::LaserShape() const
{
  return laser_shape_;
}

bool SkinSensor::intializeFromXML(const std::string& file)
{
  // parse the sdf
  sdf::ElementPtr patch_elem = this->sdf->GetElement("ray");
  sdf::ElementPtr proximity_elem = patch_elem->GetElement("range");
  min_range_ = proximity_elem->Get<double>("min");
  max_range_ = proximity_elem->Get<double>("max");

  // load the patch
  if(!server_.loadRequest(patch_xml_name_, file, min_range_, max_range_))
  {
    ROS_ERROR("SkinSensor::intializeFromXML(): parsing failed");
    return false;
  }

  // resize to number of cells
  size_ = server_.numberOfCells();                              
  cell_cloud_->resize(size_);
  normal_cloud_->resize(size_);
  cell_parent_cloud_->resize(size_);
  normal_parent_cloud_->resize(size_);
  proximity_cloud_->resize(size_);

  // get the transformation to gazebo parent body
  Eigen::Affine3f T_0_parent;
  T_0_parent.translation() << 
    this->pose.Pos().X(), this->pose.Pos().Y(), this->pose.Pos().Z();
  T_0_parent.linear() = Eigen::Quaternionf(
    this->pose.Rot().W(), this->pose.Rot().X(), this->pose.Rot().Y(), this->pose.Rot().Z()).toRotationMatrix();
  T_parent_sensor_ = T_0_parent.inverse();

  // get patch information
  const auto& positions = server_.data().positions;
  const auto& normals = server_.data().normals;
  const auto& transformations = server_.data().transformations;

  // setup the pointclouds:
  // cell_cloud: cell origins
  // normal_cloud: cell normal vectors
  Eigen::Affine3f T_cell_sensor, T_cell_parent;
  Eigen::Vector3f start_local, end_local;
  Eigen::Vector3f start_parent, end_parent;
  for(size_t i = 0; i < size_; ++i)
  {
    T_cell_sensor = transformations[i].cast<float>();
    T_cell_parent = T_0_parent*T_cell_sensor;

    start_local << 0, 0, min_range_;
    end_local << 0, 0, max_range_;

    start_parent = T_cell_parent * start_local;
    end_parent = T_cell_parent * end_local;

    cell_cloud_->at(i).getVector3fMap() = T_cell_sensor * start_local;
    normal_cloud_->at(i).getVector3fMap() = T_cell_sensor.rotation() * Eigen::Vector3f::UnitZ();

    // set ray tracer: note this needs to happen in parent frame (not the sensor frame)
    laser_shape_->AddRay(
      ignition::math::Vector3d(start_parent.x(), start_parent.y(), start_parent.z()), 
      ignition::math::Vector3d(end_parent.x(), end_parent.y(), end_parent.z()));
  }

  return true;
}

void SkinSensor::setCollisionMasks()
{
  // set the collision map to ignore the parentsId (no self collisions)
  //if(parent_collision_)
  //  parent_collision_->SetCollideBits(parentId);
  laser_collision_->SetCategoryBits(GZ_SENSOR_COLLIDE);
  laser_collision_->SetCollideBits(0x0000); // ~parentId 
}

double SkinSensor::distanceToProximity(double x)
{ 
  return std::min(1.073, 1.8947770123672258 * std::exp(-233.3923551400172 * x) + 0.02850517563038315);
}