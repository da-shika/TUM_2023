#ifndef SKIN_SKIN_MODEL_H_
#define SKIN_SKIN_MODEL_H_

////////////////////////////////////////////////////////////////////////////////
// skin_client 
////////////////////////////////////////////////////////////////////////////////
#include <skin_client/patch_client_factory.h>

////////////////////////////////////////////////////////////////////////////////
// pinocchio  
////////////////////////////////////////////////////////////////////////////////
#include <pinocchio/multibody/model.hpp>

////////////////////////////////////////////////////////////////////////////////
// control_core  
////////////////////////////////////////////////////////////////////////////////
#include <control_core/interfaces/module_base.h>
#include <control_core/types.h>

namespace skin
{

  /**
   * @brief Kinematic model of skin
   * 
   * Computes the placement of skin cells and their
   * velocity. Adds surface frames to the robot model
   * at key patches.
   */
  class SkinModel : public cc::ModuleBase
  {
    public:
      typedef cc::ModuleBase Base;
      typedef skin_client::SkinPatchClient::Ptr Client;
      typedef std::vector<Client> Clients;
      typedef skin_client::SkinPatchClient::Data Data;

      typedef pinocchio::FrameIndex FrameIndex;

    private:
      // configurations
      std::vector<std::string> skin_configs_;                                   // skin config file names

      // skin
      size_t n_cells_;
      Clients clients_;                                                         // skin clients

      // robot model
      FrameIndex base_frame_id_;
      std::vector<FrameIndex> client_ids_;
      pinocchio::Model& model_;

      // suface_frame_names frames
      std::vector<std::string> surface_frame_names_;                            // virtual frame names
      std::vector<tf::StampedTransform> surface_frame_tfs_;

      // data
      std_msgs::Header header_;
      Data::CellCloud::Ptr merged_cell_cloud_;                                  // merged cell cloud
      std::vector<cc::Scalar> velocities_; 

      // jacobians
      pinocchio::SE3 X_c_b_;
      pinocchio::SE3 placement_;
      pinocchio::Motion v_joint_, drift_b_;
      pinocchio::Data::Matrix6x J_cell_;

    public:
      SkinModel(
        const std::string& name,
        pinocchio::Model& model,
        const std::vector<std::string>& skin_configs = {});
      virtual ~SkinModel();
      
      //////////////////////////////////////////////////////////////////////////
      // update
      //////////////////////////////////////////////////////////////////////////

      /**
       * @brief compute cell positions and velocites
       */
      void updateAll(
        const cc::VectorX& q,
        const cc::VectorX& v,
        pinocchio::Data & data, 
        const ros::Time& time);

      /**
       * @brief compute cell positions
       */
      void updateCellPlacement(
        pinocchio::Data& data, 
        const ros::Time& time);
      
      /**
       * @brief compute cell velocites
       */
      void udpateCellVelocity(
        const cc::VectorX& v,
        pinocchio::Data& data, 
        const ros::Time& time);

      //////////////////////////////////////////////////////////////////////////
      // results
      //////////////////////////////////////////////////////////////////////////
      
      /**
       * @brief access to merged cell cloud in base frame
       */
      Data::CellCloud::ConstPtr cellCloud() const { return merged_cell_cloud_; }

      /**
       * @brief access to cell wise normal velocites
       */
      const std::vector<cc::Scalar>& cellVelocities() const { return velocities_; }

      /**
       * @brief total number of cells
       */
      size_t numCells() const { return n_cells_; }

      /**
       * @brief skin surface names
       */
      const std::vector<std::string>& surfaceFrameNames() const { return surface_frame_names_; }
      const std::vector<tf::StampedTransform>& surfaceFrameTfs() const { return surface_frame_tfs_; }

    protected:
      virtual bool init(ros::NodeHandle& nh, Parameters& params);

      /**
       * @brief add key frames to the surface of patches
       */
      bool addSurfaceFrames(cc::Parameters& parameter, const std::string& ns);
  };
}

#endif