#include <skin_client_real/skin_client_real.h>
#include <skin_client/patch_client_factory.h>

#include <control_core/ros/ros.h>
#include <control_core/ros/marker.h>

namespace skin_client_real
{

  SkinClientReal::SkinClientReal(const std::string& name, const std::vector<std::string>& configs) : 
      Base(name), 
      configs_(configs),
      is_valid_(false)
  {
  }

  SkinClientReal::~SkinClientReal()
  {
  }

  bool SkinClientReal::init(ros::NodeHandle& nh, cc::Parameters& params)
  {
    ////////////////////////////////////////////////////////////////////////////
    // load parameters
    ////////////////////////////////////////////////////////////////////////////
    if(!params_.fromParamServer(nh, Base::name()))
    {
      ROS_ERROR_STREAM("SkinClientReal::init: Failed to init SkinClientReal");
      return false;
    }

    if(configs_.empty())
    {
      ROS_ERROR_STREAM("SkinClientReal::init: no configs");
      return false;
    }
    configs_ = skin_client::sort_patch_names(configs_);

    ////////////////////////////////////////////////////////////////////////////
    // init all connections
    ////////////////////////////////////////////////////////////////////////////
    for(auto config : configs_)
    {
      auto client = std::make_shared<tum_ics_skin_descr::Patch::TfMarkerDataPatch>();
      auto server = std::make_shared<skin_client::SkinPatchServer>();
      cc::SkinPatch patch = cc::SkinPatch::Zero();

      // load the server
      if(!server->loadFileRequest(config, 0.0, 1.0, true))
      {
        ROS_WARN_STREAM("SkinClientReal::init:" << " can not load server: '" << config << "' skipping patch");
        continue;
      }
      patch.frame() = server->jointFrame();

      //////////////////////////////////////////////////////////////////////////
      // load the real skin clien
      //////////////////////////////////////////////////////////////////////////
      if(!client->load(QString::fromStdString(config)))
      {
        ROS_WARN_STREAM("SkinClientReal::init:" << " can not load client: '" << config << "' skipping patch");
        continue;
      }
      if(!client->createDataConnection())
      {
        ROS_WARN_STREAM("SkinClientReal::init:" << " can not create client connection: '" << config << "' skipping patch");
        continue;
      }
      
      //////////////////////////////////////////////////////////////////////////
      // add them
      //////////////////////////////////////////////////////////////////////////
      servers_.push_back(server);
      clients_.push_back(client);
      masked_proximity_cells_.push_back(std::vector<bool>(server->numberOfCells(), false));
    }

    if(clients_.empty() || servers_.empty())
    {
      ROS_WARN_STREAM("SkinClientReal::init: no valid clients/servers");
      return false;
    }
  
    ////////////////////////////////////////////////////////////////////////////
    // mask configured cells
    ////////////////////////////////////////////////////////////////////////////
    maskCells(params_);

    ////////////////////////////////////////////////////////////////////////////
    // start everything
    ////////////////////////////////////////////////////////////////////////////

    // start all servers
    number_of_cells_ = 0;
    for(auto server : servers_)
    {
      server->enableDataConnection(nh);
      number_of_cells_ += server->numberOfCells();
    }

    // start all clients
    for(auto client : clients_)
    {
      client->enableDataConnection();
    }

    prev_time_ = ros::Time::now();
    ROS_INFO("SkinClientReal::init(): number_of_cells=%ld", number_of_cells_);
    ROS_INFO("Ready");
    return true;
  }

  void SkinClientReal::start(const ros::Time& time)
  {
    size_t num_of_masked_cells = 0, min_distance_client = 0;
    double min_distance = 1e10, max_proximity = 0.0;
    
    ////////////////////////////////////////////////////////////////////////////
    // accumulate faulty cells by checking their measurement over some time
    ////////////////////////////////////////////////////////////////////////////
    ros::Time start_time = time;
    while((ros::Time::now() - start_time) < ros::Duration(4.0))
    {
      num_of_masked_cells = 0;
      min_distance_client = 0;
      min_distance = 1e10;
      max_proximity = 0.0;

      size_t i = 0;
      for(auto client : clients_)
      {
        QVector<Skin::Cell::Data> measurments = client->data();
        auto& mask = masked_proximity_cells_[i];

        double p_cell, f_cell, d_cell_hat;
        for(size_t k = 0; k < measurments.size(); ++k)
        {
          Skin::Cell::Data& cell_measurments = measurments[k];
          p_cell = params_.proximity_gain*cell_measurments[0];
          f_cell = params_.force_gain*(cell_measurments[1] + cell_measurments[2] + cell_measurments[3])/3.0;
          d_cell_hat = proximityToDistance(p_cell);

          // check if for any cell the start length is to low
          if(d_cell_hat < params_.dead_cell_dist_thres || f_cell > params_.dead_cell_force_thes)
          {
            mask[k] = true;
            num_of_masked_cells++;
          }
          // check if we got valid data
          if(d_cell_hat < min_distance)
          {
            max_proximity = p_cell;
            min_distance_client = i;
            min_distance = d_cell_hat;
          }
        }
        i++;
      }

      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }

    ROS_INFO("SkinClientReal::start(): Masked out %ld cells of %ld cells", 
      num_of_masked_cells, number_of_cells_);
    ROS_INFO("SkinClientReal::start(): Minimum dist=%f on patch '%s'", 
      min_distance, servers_[min_distance_client]->name().c_str());

    if(num_of_masked_cells > params_.max_num_of_masked_cells)
    {
      ROS_ERROR("SkinClientReal::start(): Masked: %ld cells > max num: %d", 
        num_of_masked_cells, params_.max_num_of_masked_cells);
      is_valid_ = false;
    } 
    is_valid_ = true;
  }

  bool SkinClientReal::update(const ros::Time& time, const ros::Duration& dt)
  {
    for(size_t i = 0; i < clients_.size(); ++i)
    {
      auto client = clients_[i];
      auto server = servers_[i];

      const auto& mask = masked_proximity_cells_[i];

      // get the real measurements
      QVector<Skin::Cell::Data> measurments = client->data();

      // server data structure
      skin_client::SkinPatchData& data = server->dynamicData();

      // compute the patch data
      update_patch(data, measurments, mask);

      // forward skindata
      server->publish();
    }
    return is_valid_;
  }

  void SkinClientReal::update_patch(
    skin_client::SkinPatchData& data,
    QVector<Skin::Cell::Data>& measurments, 
    const std::vector<bool>& mask)
  {
    double p_cell, f_cell, d_cell_hat;
    for(size_t i = 0; i < measurments.size(); ++i)
    {
      Skin::Cell::Data& cell_measurments = measurments[i];

      //////////////////////////////////////////////////////////////////////////
      // get measurements
      //////////////////////////////////////////////////////////////////////////
      p_cell = params_.proximity_gain*cell_measurments[0];
      f_cell = params_.force_gain*(cell_measurments[1] + cell_measurments[2] + cell_measurments[3])/3.0;
      
      if(f_cell < params_.force_thres)
        f_cell = 0.0;
      if(p_cell < params_.proximity_thres)
        p_cell = 0.0;
      
      //////////////////////////////////////////////////////////////////////////
      // distance (set to max if cell is masked)
      //////////////////////////////////////////////////////////////////////////
      if(!mask[i])
      {
        d_cell_hat = proximityToDistance(p_cell);
      }
      else
      {
        f_cell = 0.0;
        p_cell = 0.0;
        d_cell_hat = params_.maximum_distance;
      }

      //////////////////////////////////////////////////////////////////////////
      // store data
      //////////////////////////////////////////////////////////////////////////
      data.force[i] = f_cell;
      data.prox[i] = p_cell;
      data.dist[i] = d_cell_hat;
    }
  }

  double SkinClientReal::proximityToDistance(double proximity)
  {
    return params_.maximum_distance * std::exp(
      params_.f_transf_exp * (proximity - params_.f_transf_offset));
  }

  bool SkinClientReal::maskCells(cc::Parameters& parameter)
  {
    std::string masked_ns = "masked";
    std::vector<std::string> child_names = cc::find_sub_names_spaces(masked_ns);

    for(const auto& name : child_names)
      parameter.addRequired<cc::VectorXi>(masked_ns + "/" + name);

    if(!parameter.load())
    {
      PRINT_ERROR("Error loading parameters");
      return false;
    }
    
    for(size_t i = 0; i < servers_.size(); ++i)
    {
      auto server = servers_[i];
      auto& mask = masked_proximity_cells_[i];
      if(cc::has(child_names, server->name()))
      {
        cc::VectorXi masked_ids = parameter.get<cc::VectorXi>(masked_ns + "/" + server->name());
        PRINT_INFO_STREAM("for " << server->name() << " [" << masked_ids.transpose() << "]");
        for(size_t k = 0; k < masked_ids.size(); ++k)
          if(masked_ids[k] < 0)
            std::fill(mask.begin(), mask.end(), true);
          else if(masked_ids[k] < mask.size())
            mask[masked_ids[k]] = true;
      } 
    }

    

    return true;
  }

}