#ifndef SKIN_CLIENT_FACTORY_H_
#define SKIN_CLIENT_FACTORY_H_

#include <skin_client/patch_client.h>

namespace skin_client
{
  
  /**
   * @brief list all available patches
   * 
   * @return std::vector<std::string> 
   */
  inline std::vector<std::string> list_available_patches(const std::string& ns="")
  {
    std::vector<std::string> names;

    std::string ns_ = ns;
    if(ns.empty() || ns[0] != '/')
      ns_.insert(0, "/");

    // check the available topics published by servers
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    for(auto info : master_topics)
    {
      if(info.datatype == "skin_client/SkinPatchData" && startsWith(info.name, ns_) && endsWith(info.name, "data"))
      {
        std::vector<std::string> tokens = split(info.name, '/');
        if(tokens.size() > 2)
        {
          names.push_back(tokens[tokens.size()-2]);
        }
      }
    }

    // sort them based on names
    names = sort_patch_names(names);
    return names;
  }

  /**
   * @brief loads client based on patch name
   * 
   * @param patch_name 
   * @param verbose 
   * @return skin_client::SkinPatchClient::Ptr 
   */
  inline skin_client::SkinPatchClient::Ptr load_client(
    const std::string& patch_name, bool verbose=false)
  {
    std::string xml_file_str;
    if(ros::param::get("/" + patch_name, xml_file_str))
    {
      auto client = std::make_shared<skin_client::SkinPatchClient>();
      if(!client->loadRequest(patch_name, xml_file_str, 0.0, 1.0, verbose))
      {
        ROS_WARN("skin_client::load_clients(): can not load: '%s', skipping patch", patch_name.c_str());
        return nullptr;
      }
      return client;
    }
    ROS_WARN("skin_client::load_clients(): can not load: '%s' because config not on parmeter server", patch_name.c_str());
    return nullptr;
  }

  /**
   * @brief Loads clients by checking available ros topics published by
   * servers.
   * 
   * @param clients 
   * @param total_number_cells 
   * @return true 
   * @return false 
   */
  inline bool load_clients(
    std::vector<skin_client::SkinPatchClient::Ptr>& clients,
    size_t& total_number_cells, 
    const std::string& ns="",
    bool verbose=false)
  {
    auto patch_names = list_available_patches(ns);

    // build clients based on xml in parameter server
    clients.clear();
    total_number_cells = 0;
    std::string xml_file_str;
    for(auto patch_name : patch_names)
    {
      auto client = load_client(patch_name, verbose);
      if(client)
      {
        clients.push_back(client);
        total_number_cells += client->numberOfCells();
      }
    }

    if(verbose)
      ROS_INFO("skin_client::load_clients(): '%s' Load %ld clients with %ld cells", ns.c_str(), clients.size(), total_number_cells);

    // check if we got something
    if(clients.empty())
      return false;
    return true;
  }

  /**
   * @brief list all frames that have skin patches (multiples) attached to them
   * 
   * @return std::vector<std::string> 
   */
  inline std::vector<std::string> list_available_frames(const std::string& ns="")
  {
    std::vector<std::string> frames;
    std::vector<skin_client::SkinPatchClient::Ptr> clients;
    size_t total_number_cells;
    if(!load_clients(clients, total_number_cells, ns))
      return frames;
  
    for(auto client : clients)
    {
      frames.push_back(client->jointFrame());
    }
    frames.erase( std::unique(frames.begin(), frames.end()), frames.end());
    return frames;
  }

}

#endif