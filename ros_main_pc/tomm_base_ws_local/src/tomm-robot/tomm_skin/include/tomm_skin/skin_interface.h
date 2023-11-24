#pragma once

#include <control_core/types.h>
#include <control_core/ros/ros.h>
#include <tum_ics_skin_descr/Patch/TfMarkerDataPatch.h>
#include <control_core_msgs/SkinPatches.h>
#include <geometry_msgs/PoseArray.h>

namespace tomm_skin
{
  class TommSkin
  {
  private:
    std::string arm_side_; // left / right
    std::string prefix_;   // l / r

    double f_cell_thresh_, f_patch_thresh_;
    double p_cell_thresh_, p_patch_thresh_;

    tum_ics_skin_descr::Patch::TfMarkerDataPatch patch_uArm_, patch_lArm_, patch_hand_;
    QVector<Eigen::Affine3d> tfs_patchU_dh1_, tfs_patchL_dh2_, tfs_patchH_ee_;

    // SkinPatch
    cc::SkinPatch upper_arm_, lower_arm_, hand_;

    // ros
    ros::NodeHandle nh_;
    ros::Publisher skin_patches_pub_;
    ros::Publisher hand_patch_pub_;
    ros::Publisher cell_poses_pub_;

  public:
    TommSkin(const std::string &arm_side);
    virtual ~TommSkin();

    bool config();

    void updateAll();

  private:
    bool initalizeSkinPatch(const std::string &config,
                            tum_ics_skin_descr::Patch::TfMarkerDataPatch &patch,
                            QVector<Eigen::Affine3d> &transformations);

    void updatePatch(const QVector<Skin::Cell::Data> &data,
                     const QVector<Eigen::Affine3d> &tfs_cell_dh,
                     cc::SkinModality &force,
                     cc::SkinModality &proximity,
                     bool is_hand_patch);

    void updateLowerArm();

    void updateUpperArm();

    void updateHand();

    void cellVisualizer(const QVector<Skin::Cell::Data> &data,
                        const QVector<Eigen::Affine3d> &tfs_cell_dh,
                        const std::string &frame_id);
  };
}