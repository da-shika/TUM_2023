#ifndef SKIN_CLIENT_PATCH_BASE_H_
#define SKIN_CLIENT_PATCH_BASE_H_

#include <skin_client/utilities.h>
#include <ros/ros.h>
#include <skin_client/SkinPatchData.h>

namespace skin_client
{

  struct SkinPatchDataContainerBase
  {
    typedef std::vector<Eigen::Affine3d> Transformations;
    typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
    typedef std::vector<int> Indices;

    std::string frame;                        // frame      

    Indices ids;                              // unique cell ids = patch_id*10 + cell_id

    // geometry
    Transformations transformations;          // transformation with respect to robots joint frame
    Matrix3x positions;                       // cell positions with respect to robots joint frame
    Matrix3x normals;                         // normal positions with respect to robots joint frame

    // connections
    std::vector<Indices> neighbors;           // neighboring indices
  };

  /**
   * @brief SkinPatchBase class
   * 
   * Loads skin data from xml file.
   * Sets up data containers.
   * Base for client and server.
   * 
   */
  template<typename Data>
  class SkinPatchBase
  {
  public:
    enum State {CONSTRUCTED, LOADED, CONNECTED};

  protected:
    std::string patch_name_;                      // patch identifier
    State state_;                                 // patchbase class state
    SkinPatchConfig config_data_;                 // configuration data from xml file
    Data data_;                                   // data

  public:
    SkinPatchBase() : state_(CONSTRUCTED)
    {
    }
    virtual ~SkinPatchBase()
    {
    }

    /**
     * @brief check if configuration has been loaded
     * 
     * @return true 
     * @return false 
     */
    bool isLoaded() const
    {
      return (state_ == LOADED) || (state_ == CONNECTED);
    }

    /**
     * @brief number of cells in this patch
     * 
     * @return size_t 
     */
    size_t numberOfCells() const
    {
      return config_data_.cells.size();
    }

    /**
     * @brief id of this patch
     * 
     * @return int 
     */
    int id() const
    {
      return config_data_.id;
    }

    /**
     * @brief joint frame patch is attached to
     * 
     * @return const std::string& 
     */
    const std::string& jointFrame() const
    {
      return config_data_.base_frame;
    }

    /**
     * @brief topic prefix
     * 
     * @return const std::string& 
     */
    const std::string& topicPrefix() const
    {
      return config_data_.topic_prefix;
    }

    /**
     * @brief patch name
     * 
     * @return const std::string& 
     */
    const std::string& name() const
    {
      return patch_name_;
    }

    /**
     * @brief topic name
     * 
     * @return std::string 
     */
    std::string topicName()
    {
      return "/" + topicPrefix() + "/" + patch_name_;
    }

    /**
     * @brief load configuration from file
     * 
     * @param patch_name 
     * @param file_content 
     * @param d_min 
     * @param d_max 
     * @param verbose 
     * @return true 
     * @return false 
     */
    bool loadFileRequest(
      const std::string path, 
      double min_range=0.0, double max_range=1.0, bool verbose=false)
    {
      // get the patch name from file name
      std::string patch_name = skin_client::patch_name_from_file_url(path);
      if(patch_name.empty())
      {
        ROS_WARN_STREAM("SkinDataClient::loadFileRequest:" << " invalid path name: '" << path << "'");
        return false;
      }
      patch_name_ = patch_name;

      // open document
      pugi::xml_document doc;
      pugi::xml_parse_result result = doc.load_file(path.c_str());
      if (!result)
      {
        ROS_ERROR("SkinPatchBase::load(): Failed opening patch '%s'", patch_name.c_str());
        return false;
      }

      // setup configuration
      pugi::xml_node patch_node = doc.child("SkinConfig");
      if(!parse_patch(patch_node, config_data_))
      {
        ROS_ERROR("SkinPatchBase::load(): Failed parsing patch '%s'", patch_name.c_str());
        return false;
      }

      // setup the data structure
      setupData(min_range, max_range);

      if(verbose)
      {
        ROS_INFO("'%s': numberOfCells()=%ld", name().c_str(), numberOfCells());
        ROS_INFO("'%s': jointFrame()='%s'", name().c_str(), jointFrame().c_str());
        ROS_INFO("'%s': topicPrefix()='%s'", name().c_str(), topicPrefix().c_str());
      }

      // call derived class
      if(!load(min_range, max_range, verbose))
      {
        ROS_ERROR("SkinPatchBase::load(): Failed load derived class");
        return false;
      }

      state_ = LOADED;
      return true;
    }

    /**
     * @brief load configuration from string
     * 
     * @param patch_name 
     * @param file_content 
     * @param d_min 
     * @param d_max 
     * @param verbose 
     * @return true 
     * @return false 
     */
    bool loadRequest(
      const std::string patch_name, 
      const std::string file_content, 
      double min_range=0.0, double max_range=1.0, bool verbose=false)
    {
      patch_name_ = patch_name;

      // open document
      pugi::xml_document doc;
      pugi::xml_parse_result result = doc.load(file_content.c_str());
      if (!result)
      {
        ROS_ERROR("SkinPatchBase::load(): Failed opening patch '%s'", patch_name.c_str());
        return false;
      }

      // setup configuration
      pugi::xml_node patch_node = doc.child("SkinConfig");
      if(!parse_patch(patch_node, config_data_))
      {
        ROS_ERROR("SkinPatchBase::load(): Failed parsing patch '%s'", patch_name.c_str());
        return false;
      }

      // setup the data structure
      setupData(min_range, max_range);

      if(verbose)
      {
        ROS_INFO("'%s': numberOfCells()=%ld", name().c_str(), numberOfCells());
        ROS_INFO("'%s': jointFrame()='%s'", name().c_str(), jointFrame().c_str());
        ROS_INFO("'%s': topicPrefix()='%s'", name().c_str(), topicPrefix().c_str());
      }

      // call derived class
      if(!load(min_range, max_range, verbose))
      {
        ROS_ERROR("SkinPatchBase::load(): Failed load derived class");
        return false;
      }

      state_ = LOADED;
      return true;
    }

    /**
     * @brief subscribe to skin publisher
     * 
     * @param nh 
     * @return true 
     * @return false 
     */
    virtual bool enableDataConnection(ros::NodeHandle& nh) = 0;

    const Data& data() const
    {
      return data_;
    }

    Data& data()
    {
      return data_;
    }

  protected:
    virtual bool load(
      double min_range=0.0, double max_range=1.0, bool verbose=false) = 0;

  private:
    void setupData(double min_range, double max_range)
    {
      // compute static transformations, positions, normals, ...
      Eigen::Vector3d start_local(0, 0, min_range);
      Eigen::Affine3d T_cell_joint;

      data_.ids.resize(numberOfCells());
      data_.transformations.resize(numberOfCells());
      data_.positions.resize(3, numberOfCells());
      data_.normals.resize(3, numberOfCells());
      data_.neighbors.resize(numberOfCells());
      
      size_t i = 0;
      for(auto& cell : config_data_.cells)
      {
        // note: assumes patch ids are large 100 - 900
        data_.ids[i] = 10*config_data_.id + cell.id;

        // neighboar cells
        for(auto& neighbor_id : cell.neighbors)
        {
          int idx = id_to_idx(config_data_.cells, neighbor_id);
          if(idx >= 0)
            data_.neighbors[i].push_back(idx);
        }

        // transformation
        T_cell_joint = config_data_.T_base_0*config_data_.T_root_base*cell.T_cell_root;
        data_.transformations[i] = T_cell_joint;

        // pos, normal
        data_.positions.col(i) = T_cell_joint * start_local;
        data_.normals.col(i) = T_cell_joint.rotation() * Eigen::Vector3d::UnitZ();

        i++;
      }
    }

  };

}

#endif