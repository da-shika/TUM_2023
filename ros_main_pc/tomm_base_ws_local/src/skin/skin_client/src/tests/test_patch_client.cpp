#include <skin_client/patch_client.h>

bool test_patch_client(const std::string& xml_path)
{
  skin_client::SkinPatchClient client;

  if(!client.loadFileRequest(xml_path, 0.0, 1.0, true))
  {
    ROS_ERROR("test_patch_client(): Failed loading patch");
    return false;
  }

  for(size_t i = 0; i< client.data().neighbors[0].size(); ++i)
  {
    int idx = client.data().neighbors[0][i];
    ROS_INFO("neighbors [0] : [%d] (id=%d)", idx, client.data().ids[idx]);
  }

  return true;
}

int main(int argc, char **argv)
{   
  std::string xml_path;
  if(argc > 1)
    xml_path = argv[1];
  else
    xml_path = "/home/simon/ros/workspaces/h1/h1_base_ws/src/h1-robot/tum_ics_h1_configs/SkinConfigs/p104e.xml";

  test_patch_client(xml_path);

  return 0;
}