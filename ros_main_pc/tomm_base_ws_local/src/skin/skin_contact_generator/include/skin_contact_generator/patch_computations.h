#ifndef SKIN_CONTACT_GENERATOR_PATCH_COMPUTATIONS_H_
#define SKIN_CONTACT_GENERATOR_PATCH_COMPUTATIONS_H_

#include <skin_client/patch_client.h>

#include <control_core/types.h>

namespace skin_contact_generator
{

  enum Frame {JOINT, CONTACT};

  /**
   * @brief compute contact frame based on centroid and principal axis
   */
  void computeContactFrame(
    const skin_client::SkinPatchDataContainer::CellCloud& cells, 
    const skin_client::SkinPatchDataContainer::Indices& indices,
    cc::CartesianPosition& pose);

  /**
   * @brief compute wrench, cop, number of active cells and store results in patch
   */
  void computeContactWrench(
    const skin_client::SkinPatchDataContainer::CellCloud& cells, 
    const skin_client::SkinPatchDataContainer::Indices& indices, 
    const skin_client::SkinPatchDataContainer::Measurement& force, 
    const skin_client::SkinPatchDataContainer::Measurement& proximity,
    const skin_client::SkinPatchDataContainer::Measurement& distance,
    const Frame& frame,
    cc::SkinPatch& patch);

  /**
   * @brief compute splified 2d hull of active cells
   */
  void computeContactHull(
    const skin_client::SkinPatchDataContainer::CellCloud& cells, 
    const skin_client::SkinPatchDataContainer::Indices& indices,
    const Frame& frame,
    cc::SkinPatch& patch);

}

#endif