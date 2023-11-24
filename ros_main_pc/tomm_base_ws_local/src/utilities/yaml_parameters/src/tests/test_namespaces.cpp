#include <ros/package.h>

#include <yaml_parameters/yaml_parameters.h>

int main(int argc, char **argv)
{
  std::string base_path = ros::package::getPath("yaml_parameters");
  std::string sub_path = "/configs/test.yaml";

  yaml::Parameters params;
  if (!params.loadFile(base_path + sub_path))
  {
    PRINT_ERROR("Error loading config file");
    return false;
  }

  // root namespaces
  std::vector<std::string> root_ns = params.getRootNamespaces();
  std::cout << "Root Namespaces:\n";
  for (auto ns : root_ns)
  {
    std::cout << " - " << ns << "\n";
  }

  // get sub namespaces
  std::string searched_ns = "robot"; // both with or without '/' in front are valid
  std::vector<std::string> robots = params.getSubNamespaces(searched_ns);
  std::cout << "Robots: \n";
  for (auto robot : robots)
  {
    std::cout << " - " << robot << "\n";
  }

  // change ns and get sub namespaces in deeper layers
  params.set_ns("/robot/tomm/"); // with or without '/' at front/end are valid
  searched_ns = "/frames/additional_frames/";
  std::vector<std::string> additional_frames = params.getSubNamespaces(searched_ns);
  std::cout << "Additional Frames: \n";
  for (auto frame : additional_frames)
  {
    std::cout << " - " << frame << "\n";
  }
 
  return 0;
}