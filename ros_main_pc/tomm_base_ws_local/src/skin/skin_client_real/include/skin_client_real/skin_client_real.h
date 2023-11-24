#ifndef SKIN_CLIENT_REAL_SKIN_CLIENT_REAL_H_
#define SKIN_CLIENT_REAL_SKIN_CLIENT_REAL_H_

////////////////////////////////////////////////////////////////////////////////
// ics
////////////////////////////////////////////////////////////////////////////////
#include <tum_ics_skin_descr/Patch/TfMarkerDataPatch.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types.h>
#include <control_core/interfaces/module_base.h>
#include <control_core_msgs/SkinPatches.h>

////////////////////////////////////////////////////////////////////////////////
// skin_client includes
////////////////////////////////////////////////////////////////////////////////
#include <skin_client/patch_server.h>

////////////////////////////////////////////////////////////////////////////////
// skin_client_real includes
////////////////////////////////////////////////////////////////////////////////
#include <skin_client_real/SkinClientRealParameters.h>

namespace skin_client_real
{

  /**
   * @brief SkinClientReal Class
   * 
   * Extracts basic information from skin patch (Wrench, cop, n_cells, frame)
   * and forwards it to the controller.
   * 
   * It also visualizes the information as a pointcloud and Wrench vectors.
   * 
   */
  class SkinClientReal : public cc::ModuleBase
  {
    public:
      typedef cc::ModuleBase Base;
      
      typedef std::shared_ptr<tum_ics_skin_descr::Patch::TfMarkerDataPatch> Client;
      typedef std::vector<Client> Clients;

      typedef std::shared_ptr<skin_client::SkinPatchServer> Server;
      typedef std::vector<Server> Servers;
      
    private:
      bool is_valid_;
      size_t number_of_cells_;
      ros::Time prev_time_;
      SkinClientRealParameters params_;

      //////////////////////////////////////////////////////////////////////////
      // connections includes
      //////////////////////////////////////////////////////////////////////////
      Clients clients_;    
      Servers servers_;         
      std::vector<std::string> configs_;  

      //////////////////////////////////////////////////////////////////////////
      // masked cells
      //////////////////////////////////////////////////////////////////////////
      std::vector<std::vector<bool>> masked_proximity_cells_;

    public:
      SkinClientReal(
        const std::string& name,
        const std::vector<std::string>& configs);
      
      virtual ~SkinClientReal();

      bool isValid() const { return is_valid_;}

      void start(const ros::Time& time) override;

      bool update(const ros::Time& time, const ros::Duration& dt) override;

    protected:
      virtual bool init(ros::NodeHandle& nh, cc::Parameters& global_params) override;

    private:
      bool maskCells(cc::Parameters& parameter);

      void update_patch(
        skin_client::SkinPatchData& data,
        QVector<Skin::Cell::Data>& measurments, 
        const std::vector<bool>& mask);

      double proximityToDistance(double proximity);
  };

}

#endif