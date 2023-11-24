#include <skin_client/utilities.h>
#include <iostream>

bool test_xml_loading(const std::string& xml_path)
{
  // open document
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(xml_path.c_str());
  if (!result)
  {
    std::cout << "SkinPatchBase::load(): Failed opening patch " << xml_path << std::endl;
    return false;
  }

  pugi::xml_node patch_node = doc.child("SkinConfig");
  skin_client::SkinPatchConfig config;

  if(!parse_patch(patch_node, config))
  {
    std::cout << "Failed loading xml" << std::endl;
  }
  
  std::cout << "id=" << config.id << std::endl;
  std::cout << "cells.size=" << config.cells.size() << std::endl;
  std::cout << "cell[0].id=" << config.cells[0].id << std::endl;
  for(size_t i = 0; i < config.cells[0].neighbors.size(); ++i)
    std::cout << "cell[0].neighbors[" << i << "]=" << config.cells[0].neighbors[i] << std::endl;
  std::cout << "cell[0].T_cell_root=\n" << config.cells[0].T_cell_root.matrix() << std::endl;

  return true;
}

int main(int argc, char **argv)
{   
  std::string xml_path;
  if(argc > 1)
    xml_path = argv[1];
  else
    xml_path = "/home/simon/ros/workspaces/h1/h1_base_ws/src/h1-robot/tum_ics_h1_configs/SkinConfigs/p104e.xml";

  test_xml_loading(xml_path);

  return 0;
}