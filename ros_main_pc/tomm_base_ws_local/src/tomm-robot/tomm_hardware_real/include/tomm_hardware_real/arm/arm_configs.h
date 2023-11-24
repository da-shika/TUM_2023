#include <tomm_hardware_real/utilities/joint_state.h>
#include <yaml_parameters/yaml_parameters.h>

namespace tomm_hw
{
  class ArmConfig
  {
  public:
    std::string ns;
    bool is_real_robot;
    double v_max_limit;
    double q_step_limit;
    double butterworth_f_cutoff;
    double control_period;
    JointState<DOF_ARM> js_home;
    std::vector<std::string> joint_names;
    unsigned int port;

  public:
    ArmConfig();

    bool isLoaded() const;

    bool load(const std::string &file_path);

    std::string toString() const;

  private:
    bool is_loaded_;
  };

}