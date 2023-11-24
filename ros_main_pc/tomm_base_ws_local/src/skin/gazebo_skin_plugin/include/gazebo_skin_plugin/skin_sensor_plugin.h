#ifndef GAZEBO_ROS_SKIN_PLUGIN_HH_
#define GAZEBO_ROS_SKIN_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
//#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/util/system.hh"

#include "gazebo_skin_plugin/skin_sensor.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>

#include <visualization_msgs/Marker.h>

namespace gazebo
{
  /// \brief A Ray Sensor Plugin
  class GZ_PHYSICS_VISIBLE GazeboRosSkin : public SensorPlugin
  {
    /// \brief Constructor
    public: 
      GazeboRosSkin();

      virtual ~GazeboRosSkin();

      /// \brief Load the plugin
      /// \param take in SDF root element
      void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  protected:
    bool is_first_callback_;

    physics::WorldPtr world_;
    sensors::SkinSensorPtr parent_;
    event::ConnectionPtr update_callback_;
  };
}
#endif