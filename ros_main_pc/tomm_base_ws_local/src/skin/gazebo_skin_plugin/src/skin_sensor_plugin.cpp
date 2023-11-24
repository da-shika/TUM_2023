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
#include <functional>
#include <typeinfo>

#include "gazebo/physics/physics.hh"
#include "gazebo_skin_plugin/skin_sensor_plugin.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosSkin)

/////////////////////////////////////////////////
GazeboRosSkin::GazeboRosSkin() : 
  is_first_callback_(true)
{
}

/////////////////////////////////////////////////
GazeboRosSkin::~GazeboRosSkin()
{
  this->update_callback_.reset();
  this->parent_.reset();
  this->world_.reset();
}

/////////////////////////////////////////////////
void GazeboRosSkin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  this->parent_ =
    std::dynamic_pointer_cast<sensors::SkinSensor>(_parent);
    
  if (!this->parent_) 
  {
    gzthrow("SkinPlugin requires a Skin Sensor as its parent. But got: " << typeid(_parent).name());
  }

  // turn on
  this->parent_->SetActive(true);
}