#include <gazebo/gazebo.hh>
#include <gazebo_skin_plugin/skin_sensor.h>

#include "gazebo/sensors/SensorFactory.hh"

namespace gazebo
{
  class RegisterSkinSensorPlugin : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~RegisterSkinSensorPlugin()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
      RegisterSkinSensor();
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(RegisterSkinSensorPlugin)
}