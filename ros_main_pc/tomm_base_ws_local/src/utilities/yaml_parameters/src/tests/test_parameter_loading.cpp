#include <yaml_parameters/yaml_parameters.h>
#include <ros/package.h>

int main(int argc, char **argv)
{
  std::string base_path = ros::package::getPath("yaml_parameters");
  std::string sub_path = "/configs/test.yaml";

  // ------------------- TEST 1 ------------------------------------------------
  yaml::Parameters params("left_hand_motion");
  if (!params.loadFile(base_path + sub_path))
  {
    PRINT_ERROR("Error loading config file");
    return -1;
  }

  // Check if param exists
  PRINT("Parameter [type] exists? " << params.has("type"));
  PRINT("Parameter [child_frame] exists? " << params.has("child_frame"));

  // load params
  auto type = params.get<std::string>("type");
  auto frame = params.get<std::string>("frame");
  auto kp = params.get<double>("kp", 30);
  auto weight = params.get<double>("weight", -1);

  // Note: default value needs to be const, or retrieve with
  // auto mask = params.get<Eigen::Matrix<double, 6, 1>>("mask", Eigen::Matrix<double, 6, 1>::Identity());
  const Eigen::Matrix<double, 6, 1> default_val = Eigen::Matrix<double, 6, 1>::Identity();
  auto mask = params.get<Eigen::Matrix<double, 6, 1>>("mask", default_val);

  Eigen::Quaterniond quaternion;
  params.get("quaternion", Eigen::Quaterniond::Identity(), quaternion);

  std::stringstream ss;
  ss << "type=\t" << type << "\n"
     << "frame=\t" << frame << "\n"
     << "kp=\t" << kp << "\n"
     << "weight=\t" << weight << "\n"
     << "mask=\t" << mask.transpose() << "\n"
     << "quaternion=\t" << quaternion.coeffs().transpose() << "\n";

  PRINT("Config task:\n" << ss.str());
  // ---------------------------------------------------------------------------

  // ------------------------- TEST 2 ------------------------------------------
  // Get Virtual frame names
  std::string ns = "robot/tomm/";
  yaml::Parameters robot_params(ns);
  if (!robot_params.loadFile(base_path + sub_path))
  {
    PRINT_ERROR("Error loading config file");
    return false;
  }

  std::string frame_ns = "frames/additional_frames";
  std::vector<std::string> frame_names = robot_params.getSubNamespaces(frame_ns);

  PRINT("Additional Frames:");
  // load parameters
  for (std::size_t i = 0; i < frame_names.size(); ++i)
  {

    std::string parent_name;
    Eigen::Matrix<double, 7, 1> rel_pose;

    robot_params.get(frame_ns + "/" + frame_names[i] + "/parent", parent_name);
    robot_params.get(frame_ns + "/" + frame_names[i] + "/pose", rel_pose);

    PRINT("parent_name: " << parent_name);
    PRINT("pose: " << rel_pose.transpose());;
  }
  // ---------------------------------------------------------------------------

  return 0;
}