#ifndef SKIN_CLIENT_PATCH_SERVER_H_
#define SKIN_CLIENT_PATCH_SERVER_H_

#include <skin_client/patch_base.h>

namespace skin_client
{
  /**
   * @brief SkinPatch Sever
   * 
   * This class publishes the dynamic (changing) skin information for
   * a single patch.
   */
  class SkinPatchServer : public SkinPatchBase<SkinPatchDataContainerBase>
  {
  public:
    typedef SkinPatchBase<SkinPatchDataContainerBase> Base;

  protected:
    ros::Publisher pub_;
    SkinPatchData dynamic_data_;                  // dynamic data that is recived by msg

  public:
    SkinPatchServer();
    virtual ~SkinPatchServer();

    virtual bool enableDataConnection(ros::NodeHandle& nh) override;

    void publish() const;

    uint32_t numberOfSubscriber() const;

    const SkinPatchData& dynamicData() const;

    SkinPatchData& dynamicData();

  protected:
    bool load(
      double min_range=0.0, double max_range=1.0, bool verbose=false) override;

  };

}

#endif