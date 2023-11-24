#include <tomm_hardware_real/arm/arm_configs.h>

namespace tomm_hw
{
  ArmConfig::ArmConfig()
  {
  }

  bool ArmConfig::isLoaded() const
  {
    return is_loaded_;
  }

  bool ArmConfig::load(const std::string &file_path)
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
    // ---------------- PARAMS -------------------------------------------------
    // robot type
    std::string robot_type = params.get<std::string>("robot_type", "sim");
    if (!(robot_type.compare("real")) || !(robot_type.compare("REAL")))
    {
      is_real_robot = true;
    }
    else
    {
      is_real_robot = false;
    }

    // limit and cut-off frequency
    v_max_limit = DEG2RAD(params.get<double>("v_max_limit", 60.0));
    q_step_limit = DEG2RAD(params.get<double>("q_step_limit", 5.0));

    butterworth_f_cutoff = params.get<double>("butterworth_f_cutoff", 20.0);
    control_period = params.get<double>("control_period", 2.0) / 1000.0;

    // joint names ans home posture
    params.get("joint_names", joint_names);
    VectorDOFArm joint_pos;
    params.get("home_posture", joint_pos);
    js_home.pos() = DEG2RAD(joint_pos);

    is_loaded_ = true;

    return true;
  }

  std::string ArmConfig::toString() const
  {
    std::stringstream ss;
    ss << "namespace=\t" << ns << "\n"
       << "is_real_robot=\t" << is_real_robot << "\n";

    ss << "js_home:"
       << "\n";
    for (int i = 0; i < DOF_ARM; i++)
      ss << " q[" << i << "]=\t" << js_home.q()(i) << "\n";

    ss << "v_max_limit=\t" << v_max_limit << "\n"
       << "q_step_limit=\t" << q_step_limit << "\n"
       << "butterworth_f_cutoff=\t" << butterworth_f_cutoff << "\n"
       << "control_period=\t" << control_period << "\n";

    ss << "joint_names:"
       << "\n";
    for (int i = 0; i < DOF_ARM; i++)
      ss << " q[" << i << "]=\t" << joint_names[i] << "\n";

    ss << "butterworth_f_cutoff=\t"
       << butterworth_f_cutoff << "\n";
    ss << "port=\t"
       << port << "\n";

    return ss.str();
  }
}