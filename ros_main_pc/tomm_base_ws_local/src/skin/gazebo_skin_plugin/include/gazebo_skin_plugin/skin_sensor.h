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
#ifndef GAZEBO_SKIN_PLUGIN_SkinSensor_HH_
#define GAZEBO_SKIN_PLUGIN_SkinSensor_HH_

#include <memory>
#include <string>
#include <vector>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>

#include <std_msgs/Header.h>

#include <skin_client/patch_server.h>

void RegisterSkinSensor();

namespace gazebo
{
  class OgreDynamicLines;
  class Collision;
  class MultiRayShape;

  namespace sensors
  {
    class GZ_SENSORS_VISIBLE SkinSensor: public Sensor
    {
      public:
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloud;

        typedef pcl::PointXYZ PointXYZT;
        typedef pcl::PointCloud<PointXYZT> PointXYZCloud;

      public:
        SkinSensor();
        
        virtual ~SkinSensor();

        // Documentation inherited
        virtual bool IsActive() const;

        // Documentation inherited
        virtual void Load(const std::string &_worldName);

        // Documentation inherited
        virtual void Init();

        // Documentation inherited
        virtual std::string Topic() const;

        PointCloud::ConstPtr proximityCloud();

        PointXYZCloud::ConstPtr cellCloud() const;

        PointXYZCloud::ConstPtr normalCloud() const;

        const skin_client::SkinPatchServer& patchServer() const;

        /// \brief Returns a pointer to the internal physics::MultiRayShape
        physics::MultiRayShapePtr LaserShape() const;

      protected: 
        virtual bool UpdateImpl(const bool _force);

        virtual void Fini();

        bool intializeFromXML(const std::string& file);

        void setCollisionMasks();

      private:
        double distanceToProximity(double d);

      private: 
        bool is_updated_;
        bool is_first_;

        /// \brief collision models.
        physics::CollisionPtr laser_collision_;
        physics::CollisionPtr parent_collision_;

        /// \brief Multi ray shapre pointer.
        physics::MultiRayShapePtr laser_shape_;

        /// \brief Parent entity pointer
        physics::EntityPtr parent_entity_;

        /// \brief Mutex to protect laserMsg
        std::mutex mutex_;

        /// \brief settings
        size_t size_;
        double min_range_;
        double max_range_;
        std::string patch_xml_name_;
        Eigen::Affine3f T_parent_sensor_;

        /// \brief Clouds
        std_msgs::Header header_;

        // sensor frame pointclouds
        PointCloud::Ptr proximity_cloud_;
        PointXYZCloud::Ptr cell_cloud_;
        PointXYZCloud::Ptr normal_cloud_;

        // parent frame pointclouds
        PointXYZCloud::Ptr cell_parent_cloud_;
        PointXYZCloud::Ptr normal_parent_cloud_;

        /// \brief Connections
        ros::NodeHandle nh_;
        skin_client::SkinPatchServer server_;
    };

    typedef std::shared_ptr<SkinSensor> SkinSensorPtr;
  }
}

#endif

