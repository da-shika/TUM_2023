#include "pinocchio/algorithm/frames.hpp"

#include <skin_model/skin_model.h>

#include <ics_tsid_common/utilities/pinocchio.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

namespace skin
{
  SkinModel::SkinModel(
      const std::string& name,
      pinocchio::Model& model,
      const std::vector<std::string>& skin_configs) : 
    Base(name),
    model_(model),
    skin_configs_(skin_configs)
  {
  }

  SkinModel::~SkinModel()
  {
  }

  bool SkinModel::init(ros::NodeHandle& nh, Parameters& global_params)
  {
    ////////////////////////////////////////////////////////////////////////////
    // load parameters
    ////////////////////////////////////////////////////////////////////////////
    cc::Parameters skin_parameter(nh, Base::name());
    skin_parameter.addOptional<std::vector<std::string> >("skin_configs", {});
    if(!skin_parameter.load())
    {
      PRINT_ERROR("Can't load SkinModel parameter");
      return false;
    }
    if(skin_configs_.empty())
      skin_parameter.get("skin_configs", skin_configs_);
    
    if(skin_configs_.empty())
    {
      PRINT_ERROR("No skin configuration files provided");
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // load static information of skin
    ////////////////////////////////////////////////////////////////////////////

    // setup static infromation within each patch
    size_t total_number_cells = 0;
    for(auto config : skin_configs_)
    {
      // load the file
      auto client = std::make_shared<skin_client::SkinPatchClient>();
      if(!client->loadFileRequest(config, 0.0, 1.0, false))
      {
        PRINT_WARN_STREAM("Can not load server: '" << config << "' skipping patch");
        continue;
      }
      
      // load the frame id
      if(!model_.existFrame(client->jointFrame()))
      {
        PRINT_WARN_STREAM("Can not find frame: '" << client->jointFrame() << "' skipping patch");
        continue;
      }
      FrameIndex id = model_.getFrameId(client->jointFrame());
      clients_.push_back(client);
      client_ids_.push_back(id);
      total_number_cells += client->numberOfCells();
    }

    ////////////////////////////////////////////////////////////////////////////
    // load surface frames into the model
    ////////////////////////////////////////////////////////////////////////////
    if(!addSurfaceFrames(skin_parameter, global_params.get<std::string>("prefix")))
    {
      PRINT_ERROR("Error adding surface frames");
    }

    ////////////////////////////////////////////////////////////////////////////
    // setup data field
    ////////////////////////////////////////////////////////////////////////////

    n_cells_ = total_number_cells;
    merged_cell_cloud_.reset(new Data::CellCloud());
    merged_cell_cloud_->points.reserve(n_cells_);
    header_.frame_id = "base_link";

    velocities_.resize(n_cells_);
    X_c_b_.setIdentity();
    placement_.setIdentity();
    J_cell_.setZero(6, model_.nv);

    // store the baseframe id
    base_frame_id_ = model_.getFrameId("base_link");

    return true;
  }

  void SkinModel::updateAll(
      const cc::VectorX& q,
      const cc::VectorX& v,
      pinocchio::Data & data, 
      const ros::Time& time)
  {
    updateCellPlacement(data, time);
    udpateCellVelocity(v, data, time);
  }

  void SkinModel::updateCellPlacement(
    pinocchio::Data& data, 
    const ros::Time& time)
  {
    Data::CellCloud transf_cell_cloud;
    
    // get base transformation
    const auto& frame_b = model_.frames[base_frame_id_];
    auto T_b_w = data.oMi[frame_b.parent].act(frame_b.placement);

    // transform all skin data into base frame
    pinocchio::Frame frame_j;
    pinocchio::SE3 T_j_w, T_j_b;
    tf::Transform tf_T_j_b;
    tf::Matrix3x3 tf_R;

    merged_cell_cloud_->clear();
    for(size_t i = 0; i < clients_.size(); ++i)
    {
      auto client = clients_[i];

      // get transformation wrt base
      frame_j = model_.frames[client_ids_[i]];
      T_j_w = data.oMi[frame_j.parent].act(frame_j.placement);
      T_j_b = T_b_w.actInv(T_j_w);
      
      // tf
      tf::matrixEigenToTF(T_j_b.rotation(), tf_R);
      tf_T_j_b.setOrigin(tf::Vector3(
        T_j_b.translation().x(), T_j_b.translation().y(), T_j_b.translation().z()));
      tf_T_j_b.setBasis(tf_R);

      // apply on all points and merge
      pcl_ros::transformPointCloudWithNormals(*client->data().cell_cloud, transf_cell_cloud, tf_T_j_b);
      merged_cell_cloud_->insert(merged_cell_cloud_->end(), transf_cell_cloud.begin(), transf_cell_cloud.end());
    }

    header_.stamp = ros::Time::now();
    pcl_conversions::toPCL(header_, merged_cell_cloud_->header);
  }

  void SkinModel::udpateCellVelocity(
    const cc::VectorX& v,
    pinocchio::Data& data, 
    const ros::Time& time)
  {
    typedef typename pinocchio::Data::Matrix6x::ColXpr ColXprIn;
    typedef const pinocchio::MotionRef<ColXprIn> MotionIn;
    typedef typename pinocchio::Data::Matrix6x::ColXpr ColXprOut;
    typedef pinocchio::MotionRef<ColXprOut> MotionOut;

    size_t k = 0;
    for(size_t i = 0; i < clients_.size(); ++i)
    {
      // get data
      Client client = clients_[i];
      const auto& points = client->data().positions;
      const auto& normals = client->data().normals;
      pinocchio::Index frame_id = client_ids_[i];
      const pinocchio::Frame& frame = model_.frames[frame_id];
      const pinocchio::JointIndex joint_id = frame.parent;

      // frame placement
      typename pinocchio::Data::SE3 & oMframe = data.oMf[frame_id];
      oMframe = data.oMi[joint_id] * frame.placement;

      // update cells
      for(size_t j = 0; j < client->numberOfCells(); ++j)
      {
        auto n = normals.col(j);
        auto p = points.col(j);

        X_c_b_.translation(p);
        placement_ = oMframe * X_c_b_;

        // build the jacobain to the cell frame
        J_cell_.setZero();
        pinocchio::details::translateJointJacobian(
          model_, data, joint_id, pinocchio::LOCAL, placement_, data.J, J_cell_);

        // normal velocity component
        velocities_[k] = (n.transpose()*J_cell_.topRows(3)).dot(v);
        k++;
      }

    }
  }

  /**
   * @brief add key frames to the surface of patches
   */
  bool SkinModel::addSurfaceFrames(cc::Parameters& parameter, const std::string& ns)
  {
    // get virtual frame names
    std::string frame_ns = "surface_frames";
    std::vector<std::string> frame_names = cc::find_sub_names_spaces(frame_ns, ns);

    // add to loader
    for(std::size_t i = 0; i < frame_names.size(); ++i)
    {
      parameter.addRequired<std::string>(frame_ns + "/" + frame_names[i] + "/patch");
      parameter.addRequired<int>(frame_ns + "/" + frame_names[i] + "/cell");
    }
    
    // load
    if(!parameter.load())
    {
      PRINT_ERROR("Error loading parameters");
      return false;
    }

    // add to model
    int id;
    std::string patch_name;
    std::string parent_name;
    cc::CartesianPosition rel_pose;
    
    surface_frame_names_.clear();
    surface_frame_tfs_.clear();
    for(std::size_t i = 0; i < frame_names.size(); ++i)
    {
      const std::string& frame_name = frame_names[i];

      // extract parameter
      parameter.get(frame_ns + "/" + frame_name + "/patch", patch_name);
      parameter.get(frame_ns + "/" + frame_name + "/cell", id);

      // get the robot frame
      parent_name.clear();
      for(size_t i = 0; i < clients_.size(); ++i)
      {
        if(clients_[i]->name() == patch_name)
        {
          parent_name = clients_[i]->jointFrame();
          if(id < clients_[i]->numberOfCells())
          {
            rel_pose.linear() = clients_[i]->data().positions.col(id);
            rel_pose.angular() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),clients_[i]->data().normals.col(id));
          }
          else
          {
            PRINT_ERROR("Patch '%s' invalid cell '%d'", patch_name.c_str(), id);
            parent_name.clear();
          }
        }
      }
      if(parent_name.empty())
      {
        PRINT_ERROR("No match for patch '%s' with cell '%d', skipping", patch_name.c_str(), id);
        continue;
      }

      // create transformation relative to parent
      pinocchio::SE3 pose = pinocchio::SE3::Identity();
      pose.translation() = rel_pose.linear();
      pose.rotation() = rel_pose.angular().toRotationMatrix();
      if(!ics::add_frame_to_model(model_, parent_name, frame_name+"_link", pose))
      {
        PRINT_ERROR("Error parent frame '%s' not exsisting", parent_name.c_str());
        return false;
      }      
      
      // create a new tf
      tf::Transform transform;
      transform.setIdentity();
      transform.setOrigin(rel_pose.linear());
      transform.setRotation(rel_pose.angular());
      surface_frame_tfs_.push_back(tf::StampedTransform(
        transform, ros::Time(0), parent_name, frame_name+"_link"));

      // store virtual frame
      surface_frame_names_.push_back(frame_name+"_link");
    }

    return true;
  }

}