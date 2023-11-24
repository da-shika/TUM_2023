#include <tomm_hardware_real/omnibase/omnibase_configs.h>

namespace tomm_hw
{

  OmnibaseConfig::OmnibaseConfig() : is_loaded_(false)
  {
  }

  bool OmnibaseConfig::isLoaded() const
  {
    return is_loaded_;
  }

  bool OmnibaseConfig::load(const std::string &file_path)
  {
    // ------------- Load config file ------------------------------------------
    yaml::Parameters params;
    if (!params.loadFile(file_path))
    {
      PRINT_ERROR("ArmConfig::load(): Failed to load config file");
      return false;
    }
    ns = params.getRootNamespaces()[0];
    params.set_ns(ns);

    params.get<std::string>("interface", "eth02", interface);
    params.get<std::string>("cmd_topic", "/cmd_vel_smooth", cmd_topic);
    params.get<double>("wheel_limit", 1.0, wheel_limit);
    params.get<double>("cart_limit", 0.5, cart_limit);
    params.get<double>("radius", 0.7, radius);
    params.get<double>("cmd_acc_limit", 4.0, cmd_acc_limit);
    params.get<double>("cmd_vel_limit", 1.0, cmd_vel_limit);
    params.get("odom_pub_freq", 100.0, odom_pub_freq);
    params.get<std::string>("frame_id", "odom", frame_id);
    params.get<std::string>("child_frame_id", "base_link", child_frame_id);
    params.get("state_pub_freq", 30.0, state_pub_freq);
    params.get("pub_odom_msg", true, pub_odom_msg);
    params.get("pub_tf", true, pub_tf);
    params.get<std::string>("type", "sim", omnibase_type);
    params.get("joint_names", std::vector<std::string>({""}), joint_names);

    odom_pub_dt = 1.0 / odom_pub_freq;
    state_pub_dt = 1.0 / state_pub_freq;

    // switch between real mode and simulation mode
    if (omnibase_type == "real")
    {
      is_real_robot = true;
    }
    else
    {
      is_real_robot = false;
    }
    return true;
  }

  std::string OmnibaseConfig::toString() const
  {
    std::stringstream ss;
    ss << "is_real_robot=\t" << is_real_robot << "\n"
       << "wheel_limit=\t" << wheel_limit << "\n"
       << "cart_limit=\t" << cart_limit << "\n"
                                           "radius=\t"
       << radius << "\n"
       << "cmd_acc_limit=\t" << cmd_acc_limit << "\n"
       << "cmd_vel_limit=\t" << cmd_vel_limit << "\n"
       << "frame_id=\t" << frame_id << "\n"
       << "child_frame_id=\t" << child_frame_id << "\n"
       << "state_pub_freq=\t" << state_pub_freq << "\n"
       << "pub_odom_msg=\t" << pub_odom_msg << "\n"
       << "pub_tf=\t" << pub_tf << "\n"
       << "odom_pub_dt=\t" << odom_pub_dt << "\n"
       << "state_pub_dt=\t" << state_pub_dt << "\n"
       << "joint_names_=\t";

    for (size_t i{0}; i < joint_names.size(); ++i)
      ss << joint_names[i] << "\t";

    return ss.str();
  }
}