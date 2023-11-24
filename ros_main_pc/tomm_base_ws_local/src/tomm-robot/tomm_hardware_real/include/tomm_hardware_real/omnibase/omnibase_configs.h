#include <string>
#include <QString>
#include <QFile>
#include <QFileInfo>
#include <QSettings>

#include <yaml_parameters/yaml_parameters.h>

namespace tomm_hw
{
  class OmnibaseConfig
  {
  public:
    std::string ns;
    bool is_real_robot;
    double wheel_limit;         // vel limit single wheel
    double cart_limit;          // vel limit robot
    double radius;              // maximum radius of robot
    double cmd_acc_limit;       // cmd acceleration limit
    double cmd_vel_limit;       // cmd velocity limit
    std::string frame_id;       // odom tf frame id
    std::string child_frame_id; // base_link
    bool pub_odom_msg;          // flag, publish nav_msgs/odometry
    bool pub_tf;                // publish tf frame
    std::string interface, cmd_topic, omnibase_type;
    double odom_pub_freq, state_pub_freq;
    double state_pub_dt;
    double odom_pub_dt;
    std::vector<std::string> joint_names;

  public:
    OmnibaseConfig();

    bool isLoaded() const;

    bool load(const std::string &file_path);

    std::string toString() const;

  private:
    bool is_loaded_;
  };
}