#ifndef SKIN_CONTACT_GENERATOR_SKIN_CONTACT_GENERATOR_H_
#define SKIN_CONTACT_GENERATOR_SKIN_CONTACT_GENERATOR_H_

////////////////////////////////////////////////////////////////////////////////
// skin_client
////////////////////////////////////////////////////////////////////////////////
#include <skin_client/patch_client_factory.h>

////////////////////////////////////////////////////////////////////////////////
// ros connections
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

////////////////////////////////////////////////////////////////////////////////
// pcl includes
////////////////////////////////////////////////////////////////////////////////
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/interfaces/module_base.h>
#include <control_core/types.h>
#include <control_core_msgs/SkinPatches.h>
#include <control_core_msgs/VectorStamped.h>

#include <skin_contact_generator/SkinContactGeneratorParameters.h>

namespace skin_contact_generator
{
  //////////////////////////////////////////////////////////////////////////////
  // utility container
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief contacts measured by skin
   */
  struct SkinContact
  {
    int patch_map_id;                                         // partent skin patch map id             
    skin_client::SkinPatchClient::Data::Indices ids;          // unique cell ids in this contact (not pcl indices!)
    cc::SkinPatch patch;                                      // contact information
  };

  /**
   * @brief Skin patches that belong to same robot frame are mapped together
   */
  struct SkinPatchMap
  {
    std::string frame;                                        // frame name
    std::vector<skin_client::SkinPatchClient::Ptr> clients;   // list of clients in this map
  };

  //////////////////////////////////////////////////////////////////////////////
  // WholeBodyContactGenerator
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief WholeBodyContactGenerator Class
   * 
   * Subscribe to all skin skin servers and create contact frames at active cells clusters
   * 
   * Computations:
   *  - Merge skincells into pointcloud
   *  - Create clusters and compute prinzipal axis for contact frame
   *  - Compute proximity/force wrenches, cop, hull in that contact frame
   *  - Publishes the information as a single topic
   * 
   * Note: Each contact is linked to a joint frame on the robot
   * However, the contact information is expressed in the contact frame
   * The transformation between joint and contact frame is stored in the pose field
   * 
   */
  class WholeBodyContactGenerator : public cc::ModuleBase
  {
    public:
      typedef ModuleBase Base;
      typedef skin_contact_generator::SkinContactGeneratorParameters Params;

      typedef skin_client::SkinPatchClient::Ptr Client;
      typedef std::vector<Client> Clients;
      typedef skin_client::SkinPatchClient::Data Data;

    private:
      Params params_;
      
      //////////////////////////////////////////////////////////////////////////
      // skin connection
      //////////////////////////////////////////////////////////////////////////
      Clients clients_;
      std::vector<SkinPatchMap> skinpatch_maps_;
      
      //////////////////////////////////////////////////////////////////////////
      // measurments
      //////////////////////////////////////////////////////////////////////////
      std::vector<SkinContact> contacts_;
      std::size_t n_active_cells_;

      //////////////////////////////////////////////////////////////////////////
      // ros connection
      //////////////////////////////////////////////////////////////////////////
      ros::Publisher contact_pub_;                  // contact list
      ros::Publisher n_active_cell_pub_;            // num active cells

    public:
      WholeBodyContactGenerator(const std::string& name);
      virtual ~WholeBodyContactGenerator();

    protected:
      virtual bool init(ros::NodeHandle& nh, Base::Parameters& global_params) override;

      virtual bool update(const ros::Time &time, const ros::Duration &period) override;

    private:
      Data::Indices extractCellIds(const Data::Indices indices, const Data::Indices& ids);
  };

}

#endif