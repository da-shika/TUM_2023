#ifndef SKIN_CONTACT_GENERATOR_SKIN_CONTACT_GENERATOR_H_
#define SKIN_CONTACT_GENERATOR_SKIN_CONTACT_GENERATOR_H_

////////////////////////////////////////////////////////////////////////////////
// skin_client
////////////////////////////////////////////////////////////////////////////////
#include <skin_client/patch_client_factory.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/interfaces/module_base.h>
#include <control_core/types.h>
#include <control_core_msgs/SkinPatches.h>

#include <skin_contact_generator/SkinContactGeneratorParameters.h>

namespace skin_contact_generator
{
  class PatchContactGenerator : public cc::ModuleBase
  {
    public:
      typedef ModuleBase Base;
      typedef skin_contact_generator::SkinContactGeneratorParameters Params;
      
      typedef std::unique_ptr<skin_client::SkinPatchClient> Client;
      typedef skin_client::SkinPatchClient::Data Data;

    private:
      Params params_;
      std::string patch_name_;

      //////////////////////////////////////////////////////////////////////////
      // skin connection
      //////////////////////////////////////////////////////////////////////////
      Client client_;

      //////////////////////////////////////////////////////////////////////////
      // contact information
      //////////////////////////////////////////////////////////////////////////
      cc::SkinPatch patch_;
      control_core_msgs::SkinPatch patch_msg_;

      //////////////////////////////////////////////////////////////////////////
      // ros connection
      //////////////////////////////////////////////////////////////////////////
      ros::Publisher patch_pub_;

    public:
      PatchContactGenerator(const std::string& name, const std::string& patch_name);
      virtual ~PatchContactGenerator();

    protected:
      virtual bool init(ros::NodeHandle& nh, Base::Parameters& global_params) override;

      virtual bool update(const ros::Time &time, const ros::Duration &period) override;
  };
}

#endif