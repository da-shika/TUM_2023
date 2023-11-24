#include <tomm_skin/skin_interface.h>

using namespace tomm_skin;

TommSkin::TommSkin(const std::string &arm_side) : arm_side_{arm_side},
                                                  prefix_{arm_side_[0]},
                                                  f_cell_thresh_{0.015},
                                                  f_patch_thresh_{0.05},
                                                  p_cell_thresh_{0.015},
                                                  p_patch_thresh_{0.05}
{
  upper_arm_.frame() = prefix_ + "_upper_arm_link";
  lower_arm_.frame() = prefix_ + "_forearm_link";
  hand_.frame() = "hand_link_" + arm_side_;

  std::string topic = arm_side_ + "_skin_patches";
  skin_patches_pub_ = nh_.advertise<control_core_msgs::SkinPatches>(topic, 1);
  topic = arm_side_ + "_hand_patch";
  hand_patch_pub_ = nh_.advertise<control_core_msgs::SkinPatches>(topic, 1);

  cell_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("cell_frames", 1);
}

TommSkin::~TommSkin()
{
}

void TommSkin::updateAll()
{
  control_core_msgs::SkinPatches skin_patches_msg;

  updateLowerArm();
  control_core_msgs::SkinPatch lower_patch_msg = lower_arm_.toSkinPatchMsg();
  lower_patch_msg.header.stamp = ros::Time::now();
  skin_patches_msg.patches.push_back(lower_patch_msg);

  updateUpperArm();
  control_core_msgs::SkinPatch upper_patch_msg = upper_arm_.toSkinPatchMsg();
  upper_patch_msg.header.stamp = ros::Time::now();
  skin_patches_msg.patches.push_back(upper_patch_msg);

  updateHand();
  control_core_msgs::SkinPatch hand_patch_msg = hand_.toSkinPatchMsg();
  hand_patch_msg.header.stamp = ros::Time::now();
  skin_patches_msg.patches.push_back(hand_patch_msg);
  skin_patches_pub_.publish(skin_patches_msg);

  control_core_msgs::SkinPatches hand_patch;
  hand_patch.patches.push_back(hand_patch_msg);
  hand_patch_pub_.publish(hand_patch);  
}

void TommSkin::updateLowerArm()
{
  cc::SkinModality lArm_force, lArm_proximity;
  updatePatch(patch_lArm_.data(), tfs_patchL_dh2_, lArm_force, lArm_proximity, false);
  lower_arm_.force() = lArm_force;
  lower_arm_.proximity() = lArm_proximity;

  cc::CartesianPosition pose;
  pose.linear() = lower_arm_.proximity().cop();
  pose.angular().setIdentity();
  lower_arm_.pose() = pose;
}

void TommSkin::updateUpperArm()
{
  cc::SkinModality uArm_force, uArm_proximity;
  updatePatch(patch_uArm_.data(), tfs_patchU_dh1_, uArm_force, uArm_proximity, false);
  upper_arm_.force() = uArm_force;
  upper_arm_.proximity() = uArm_proximity;

  cc::CartesianPosition pose;
  pose.linear() = upper_arm_.proximity().cop();
  pose.angular().setIdentity();
  upper_arm_.pose() = pose;
}

void TommSkin::updateHand()
{
  cc::SkinModality hand_force, hand_proximity;
  updatePatch(patch_hand_.data(), tfs_patchH_ee_, hand_force, hand_proximity, true);
  hand_.force() = hand_force;
  hand_.proximity() = hand_proximity;

  // cc::Matrix3 R_h_ee;
  // R_h_ee << 1, 0, 0,
  //     0, -0.7071068, 0.7071068,
  //     0, -0.7071068, -0.7071068;
  // cc::AngularPosition Q_h_ee{R_h_ee};

  // cc::CartesianPosition pose;
  // pose.linear() << 0.075, -0.0247, 0.0247;
  // pose.angular() = Q_h_ee;
  // hand_.pose() = pose;
  hand_.pose() = cc::CartesianPosition::Zero();
}

void TommSkin::updatePatch(const QVector<Skin::Cell::Data> &data,
                           const QVector<Eigen::Affine3d> &tfs_cell_dh,
                           cc::SkinModality &force,
                           cc::SkinModality &proximity,
                           bool is_hand_patch)
{
  double f_sum = 0.0, p_sum = 0.0;
  cc::LinearPosition t_sum{cc::LinearPosition::Zero()}; // for average translation of all cells
  cc::Vector3 f_weighted_sum{cc::Vector3::Zero()}, p_weighted_sum{cc::Vector3::Zero()};

  std::unordered_map<int, double> force_map;
  std::unordered_map<int, double> proximity_map;

  int nActCells_force = 0, nActCells_proximity = 0;
  for (int i{0}; i < data.size(); ++i)
  {
    // translation
    cc::LinearPosition t_cell_dh = tfs_cell_dh[i].translation();
    t_sum += t_cell_dh;

    // force
    double f_cell = 0.0;
    for (int j{0}; j < 3; ++j)
      f_cell += data[i].force[j];
    f_cell /= 3.0;

    if (f_cell > f_cell_thresh_)
    {
      ++nActCells_force;
      f_sum += f_cell;
      f_weighted_sum += f_cell * t_cell_dh;

      force_map.emplace(i, f_cell);
    }

    // proximity
    double p_cell = data[i].prox[0];
    if (p_cell > p_cell_thresh_)
    {
      ++nActCells_proximity;
      p_sum += p_cell;
      p_weighted_sum += p_cell * t_cell_dh;

      proximity_map.emplace(i, p_cell);
    }
  }

  // area
  force.area() = nActCells_force;
  proximity.area() = nActCells_proximity;

  // cop of force
  if (f_sum > f_patch_thresh_)
    force.cop() = f_weighted_sum / f_sum;
  else
    force.cop() = t_sum / data.size();

  // cop of proximity
  if (p_sum > p_patch_thresh_)
    proximity.cop() = p_weighted_sum / p_sum;
  else
    proximity.cop() = t_sum / data.size();

  if (is_hand_patch)
  {
    force.cop().setZero();
    proximity.cop().setZero();
  }

  // wrench of force
  force.wrench().setZero();
  cc::Vector3 F_cell{cc::Vector3::Zero()}, F_cell_dh{cc::Vector3::Zero()};
  for (auto &it : force_map)
  {
    int idx = it.first;
    double f = it.second;

    F_cell.z() = f;
    F_cell_dh = tfs_cell_dh[idx].rotation() * F_cell;
    force.wrench().head(3) += F_cell_dh;
    cc::LinearPosition r = tfs_cell_dh[idx].translation() - force.cop();
    force.wrench().tail(3) += r.cross(F_cell_dh);
  }

  // wrench of proximity
  proximity.wrench().setZero();
  cc::Vector3 P_cell{cc::Vector3::Zero()}, P_cell_dh{cc::Vector3::Zero()};
  for (auto &it : proximity_map)
  {
    int idx = it.first;
    double prox = it.second;

    P_cell.z() = prox;
    P_cell_dh = tfs_cell_dh[idx].rotation() * P_cell;
    proximity.wrench().head(3) += P_cell_dh;
    cc::LinearPosition r = tfs_cell_dh[idx].translation() - proximity.cop();
    proximity.wrench().tail(3) += r.cross(P_cell_dh);
  }
}

void TommSkin::cellVisualizer(const QVector<Skin::Cell::Data> &data,
                              const QVector<Eigen::Affine3d> &tfs_cell_dh,
                              const std::string &frame_id)
{
  geometry_msgs::PoseArray poses_msg;
  poses_msg.header.frame_id = frame_id;
  poses_msg.header.stamp = ros::Time::now();
  for (size_t i = 0; i < data.size(); ++i)
  {
    geometry_msgs::Pose pose;

    cc::CartesianPosition cell_pose{tfs_cell_dh[i]};
    pose = cell_pose;
    poses_msg.poses.push_back(pose);
  }

  cell_poses_pub_.publish(poses_msg);
}

bool TommSkin::config()
{
  // Load skin config files
  std::string lower_arm_config_file, upper_arm_config_file, hand_config_file;

  if (!cc::load("~" + arm_side_ + "_lower_arm_config_file", lower_arm_config_file))
  {
    ROS_ERROR_STREAM(arm_side_ << ": TommSkin::config(): error reading skin config:" << lower_arm_config_file);
    return false;
  }
  if (!cc::load("~" + arm_side_ + "_upper_arm_config_file", upper_arm_config_file))
  {
    ROS_ERROR_STREAM(arm_side_ << ": TommSkin::config(): error reading skin config:" << upper_arm_config_file);
    return false;
  }
  if (!cc::load("~" + arm_side_ + "_hand_config_file", hand_config_file))
  {
    ROS_ERROR_STREAM(arm_side_ << ": TommSkin::config(): error reading skin config:" << hand_config_file);
    return false;
  }

  // initialize skin patches
  if (!initalizeSkinPatch(lower_arm_config_file, patch_lArm_, tfs_patchL_dh2_))
  {
    ROS_ERROR_STREAM(arm_side_ << ": TommSkin::config(): error init skin_patch: " << lower_arm_config_file);
    return false;
  }
  if (!initalizeSkinPatch(upper_arm_config_file, patch_uArm_, tfs_patchU_dh1_))
  {
    ROS_ERROR_STREAM(arm_side_ << ": TommSkin::config(): error init skin_patch: " << upper_arm_config_file);
    return false;
  }
  if (!initalizeSkinPatch(hand_config_file, patch_hand_, tfs_patchH_ee_))
  {
    ROS_ERROR_STREAM(arm_side_ << ": TommSkin::config(): error init skin_patch: " << hand_config_file);
    return false;
  }

  return true;
}

bool TommSkin::initalizeSkinPatch(const std::string &config,
                                  tum_ics_skin_descr::Patch::TfMarkerDataPatch &patch,
                                  QVector<Eigen::Affine3d> &transformations)
{
  std::string err_str;

  QString skin_config_file = QString::fromStdString(config);
  if (!patch.load(skin_config_file))
  {
    ROS_ERROR_STREAM(arm_side_ << ": TommSkin::config(): error loading skin config");
    return false;
  }
  if (patch.isUndefined())
  {
    ROS_ERROR_STREAM(arm_side_ << ": TommSkin::config(): skin patch is undefined");
    return false;
  }
  patch.createDataConnection();
  patch.enableDataConnection();

  transformations = patch.tf_cell_base();

  return true;
}