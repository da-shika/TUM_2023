#include <skin_contact_generator/patch_contact_generator.h>
#include <skin_contact_generator/patch_computations.h>

namespace skin_contact_generator
{

  PatchContactGenerator::PatchContactGenerator(
      const std::string& name, const std::string& patch_name) : 
    Base(name),
    patch_name_(patch_name)
  {
  }

  PatchContactGenerator::~PatchContactGenerator()
  {
  }

  bool PatchContactGenerator::init(ros::NodeHandle& nh, Base::Parameters& global_params)
  {
    //////////////////////////////////////////////////////////////////////////
    // load parameters
    //////////////////////////////////////////////////////////////////////////
    if(!params_.fromParamServer(nh, Base::name()))
    {
      PRINT_ERROR("Error loading parameters");
      return false;
    }
    std::string xml_file_str;
    if(!cc::load("/" + patch_name_, xml_file_str))
    {
      PRINT_ERROR("Error loading xml: '%s' from parameter server", patch_name_);
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // connect to desired cell patch
    ////////////////////////////////////////////////////////////////////////////
    client_ = std::make_unique<skin_client::SkinPatchClient>();
    if(!client_->loadRequest(patch_name_, xml_file_str, 0.0, 1.0, false))
    {
      PRINT_ERROR("Can not load: '%s'", patch_name_);
      return false;
    }
    client_->enableDataConnection(nh);

    patch_pub_ = nh.advertise<control_core_msgs::SkinPatch>(Base::name() + "_patch", 1);

    patch_.frame() = client_->jointFrame();
    patch_.setZero();
    return true;
  }

  bool PatchContactGenerator::update(const ros::Time &time, const ros::Duration &period)
  {
    // extract active cells
    Data::Indices indices;
    indices = client_->computeActiveDistanceIndices(params_.active_distance);
    if(indices.empty() || patch_pub_.getNumSubscribers() == 0)
      return true;

    // create contact frame
    computeContactFrame(
      *client_->data().cell_cloud, indices, patch_.pose());

    // compute the wrenches acting at that contact, base oriented
    computeContactWrench(
      *client_->data().cell_cloud, indices, client_->data().force, client_->data().proximity, 
      client_->data().distance, CONTACT, patch_);
    
    // compute the contact hulls
    computeContactHull(*client_->data().cell_cloud, indices, CONTACT, patch_);

    // publish
    patch_msg_ = patch_;
    patch_msg_.header.stamp = time;
    patch_pub_.publish(patch_msg_);
    return true;
  }

}