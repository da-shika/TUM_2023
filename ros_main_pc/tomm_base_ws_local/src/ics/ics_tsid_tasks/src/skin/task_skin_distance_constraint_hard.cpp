#include <tsid/robots/robot-wrapper.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ics_tsid_tasks/skin/task_skin_distance_constraint_hard.h>

#include <control_core/ros/parameters.h>

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskSkinDistanceConstraintHard::TaskSkinDistanceConstraintHard(const std::string & name,
                                           cc::Scalar dt,
                                           RobotWrapper & robot):
      Base(name, robot),
      dt_(dt),
      constraint_(name)
    {
      //////////////////////////////////////////////////////////////////////////
      // setup members
      //////////////////////////////////////////////////////////////////////////
      X_c_b.setIdentity();
      placement.setIdentity();
      J_cell.setZero(6, m_robot.nv());
      num_active_ieq_ = 0;
      num_active_eq_ = 0;
      min_dist_ = 1e10;

      //////////////////////////////////////////////////////////////////////////
      // setup inequality (set to default dim=1)
      //////////////////////////////////////////////////////////////////////////
      setMask(Vector::Ones(robot.na()));
      constraint_.matrix().setZero(1, robot.nv());
      constraint_.lowerBound().setConstant(1, -1e10);
      constraint_.upperBound().setConstant(1, 1e10);

      //////////////////////////////////////////////////////////////////////////
      // setup fixed part of qp problem
      //////////////////////////////////////////////////////////////////////////
      int nv = robot.nv();
      
      a_.setZero(nv);
      Aeq_.resize(0, nv);
      beq_.resize(0, nv);
      Q_.setIdentity(nv,nv);
      p_.setZero(nv);
      
      // generator matrix
      G_ = cc::pointContactVRep(0.7).rowwise().normalized();
    }

    bool TaskSkinDistanceConstraintHard::connect(ros::NodeHandle& nh)
    {
      //////////////////////////////////////////////////////////////////////////
      // load params
      //////////////////////////////////////////////////////////////////////////
      if(!params_.fromParamServer(nh, Base::name()))
      {
        ROS_ERROR("TaskSkinDistanceConstraintHard:init(): Failed to init parameters");
        return false;
      }
      // setup parameters server
      reconfig_srv_ = std::make_unique<Server>(params_.privateNamespace());
      reconfig_srv_->setCallback(boost::bind(&TaskSkinDistanceConstraintHard::reconfigureRequest, this, _1, _2));

      //////////////////////////////////////////////////////////////////////////
      // setup clients
      //////////////////////////////////////////////////////////////////////////
      size_t total_number_cells;
      tsid::tasks::TaskSkinDistanceConstraintHard::Clients client_candidates;
      if(!skin_client::load_clients(client_candidates, total_number_cells))
      {
        ROS_WARN("TaskSkinDistanceConstraintHard:init() '%s' could not find any servers", Base::name().c_str());
      }

      //////////////////////////////////////////////////////////////////////////
      // check if frame is known, map frame id to jacobian
      //////////////////////////////////////////////////////////////////////////
      total_number_cells = 0;
      for(auto client : client_candidates)
      {
        if(cc::has(params_.masked_patches, client->name()))
        {
          ROS_WARN("TaskSkinDistanceConstraintHard:init() '%s' masked",client->name().c_str()); 
          continue;
        }
        if(m_robot.model().existFrame(client->jointFrame()))
        {
          Index id = m_robot.model().getFrameId(client->jointFrame());
          m_client_ids.push_back(id);
          clients_.push_back(client);
          patch_starts_.push_back(total_number_cells);
          patch_sizes_.push_back(client->numberOfCells());
          total_number_cells += client->numberOfCells();
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // setup inequality (set to dim=1 in case no patch could be loaded)
      //////////////////////////////////////////////////////////////////////////
      size_t total_number_generators = std::max(size_t(1), 4*total_number_cells);
      constraint_.matrix().setZero(total_number_generators, Base::m_robot.nv());
      constraint_.lowerBound().setConstant(total_number_generators, -1e10);
      constraint_.upperBound().setConstant(total_number_generators, 1e10);
      a_zero_limits_.setZero(total_number_generators);

      //////////////////////////////////////////////////////////////////////////
      // setup matrix buffers
      //////////////////////////////////////////////////////////////////////////
      int rows = total_number_generators;
      int cols = Base::m_robot.nv();
      mem_Aieq_.resize(rows*cols, 0.0);
      mem_bieq_.resize(rows, 0.0);

      //////////////////////////////////////////////////////////////////////////
      // connect them
      //////////////////////////////////////////////////////////////////////////
      ros::NodeHandle internal_nh("~");
      for(auto client : clients_)
        client->enableDataConnection(internal_nh);

      ROS_INFO_STREAM("TaskSkinDistanceConstraintHard::TaskSkinDistanceConstraintHard(): cells=" << 
        total_number_cells << " clients=" << clients_.size());

      return true;
    }

    void TaskSkinDistanceConstraintHard::setMask(ConstRefVector m)
    {
      assert(m.size()==m_robot.na());
      m_mask = m;
    }

    double TaskSkinDistanceConstraintHard::minimumDistance() const
    {
      return min_dist_;
    }

    int TaskSkinDistanceConstraintHard::numActiveConstraints() const
    {
      return num_active_ieq_;
    }

    const ConstraintBase & TaskSkinDistanceConstraintHard::compute(const double ,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
    {
      min_dist_ = 1e10;
      num_active_ieq_ = 0;
      num_active_eq_ = 0;
      
      // reset
      violated_starts_.clear();
      violated_sizes_.clear();
      violated_cells_.clear();

      //////////////////////////////////////////////////////////////////////////
      // iterate over all patches and update the constraints
      //////////////////////////////////////////////////////////////////////////
      size_t k = 0; // cell count
      for(size_t i = 0; i < clients_.size(); ++i)
      {
        // patch data
        Client client = clients_[i];
        const auto& patch_start = patch_starts_[i];
        const auto& patch_size = patch_sizes_[i];
        const auto& points = client->data().positions;
        const auto& normals = client->data().normals;
        const auto& distances = client->data().distance;
        
        // get the joint frame and corresponding chain
        Index frame_id = m_client_ids[i];
        const Frame & frame = m_robot.model().frames[frame_id];
        const JointIndex joint_id = frame.parent;
        
        // frame placement
        typename Data::SE3 & oMframe = data.oMf[frame_id];
        oMframe = data.oMi[joint_id] * frame.placement;

        // count number of active in size this patch
        num_active_cell_ = 0;
        for(size_t j = 0; j < client->numberOfCells(); ++j)
        {
          if(distances[j] < params_.d_start)
            num_active_cell_++;
        }
        bool valid = (num_active_cell_ > 2);                                    // TODO: min num of active cells
        
        valid=true;
        
        ////////////////////////////////////////////////////////////////////////
        // update cells
        ////////////////////////////////////////////////////////////////////////
        for(size_t j = 0; j < client->numberOfCells(); ++j)
        {
          const auto& n = normals.col(j);
          const auto& p = points.col(j);
          double d = distances[j];

          if(valid && (d < params_.d_start))
          {
            // this constraint has to do something
            if(d < min_dist_)
              min_dist_ = d;

            // set the placement of the cell wrt to joint
            X_c_b.translation(p);
            placement = oMframe * X_c_b;

            // build the jacobain to the cell frame
            J_cell.setZero();
            pinocchio::details::translateJointJacobian(
              m_robot.model(), data, joint_id, pinocchio::LOCAL, placement, data.J, J_cell);

            // collision space of cell k
            constraint_.matrix().row(k) = n.transpose()*J_cell.topRows(3);

            // bias acceleration in cell frame
            drift_b = X_c_b.actInv(data.a[joint_id]);
            v_joint = X_c_b.actInv(data.v[joint_id]);
            drift_b.linear() += v_joint.angular().cross(v_joint.linear());

            // dP > 0 moving into the collision, dP < 0 backing off
            double dP = constraint_.matrix().row(k).dot(v);

            // acceleration limit
            cc::Scalar fac = (d - params_.d_min) / (params_.d_start - params_.d_min);
            cc::Scalar vel_limit = std::max(-std::abs(params_.v_max), fac*params_.v_damp);
            cc::Scalar acc_limit = (vel_limit - dP) / dt_;

            a_zero_limits_[k] = (0.0 - dP) / dt_ - n.dot(drift_b.linear());
            
            // check how to handle this action
            if(d >= params_.d_start)
            {
              // this is not violated, limit velocity
              constraint_.upperBound()[k] = acc_limit - n.dot(drift_b.linear());
              num_active_ieq_++;
            }
            else
            {
              // this is violated, set velocity to zero
              constraint_.upperBound()[k] = acc_limit - n.dot(drift_b.linear());

              // resize test qp
              violated_starts_.push_back(patch_start);
              violated_sizes_.push_back(patch_size);
              violated_cells_.push_back(k);

              int new_size = num_active_eq_ + 4;
              Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned> Aieq(mem_Aieq_.data(), m_robot.nv(), new_size);
              Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> bieq(mem_bieq_.data(), new_size);

              R_c_j_ = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), n).toRotationMatrix();
              Aieq.rightCols(4) = (G_*R_c_j_.transpose()*J_cell.topRows(3)).transpose();
              bieq.tail(4).setConstant(-100.0); // constraint_.upperBound()[k]    // no need for real velocity!
              num_active_eq_ += 4;
            }
          }
          else
          {
            // deactivate
            constraint_.upperBound()[k] = 1e10;
          }

          k++;
        }
      }

      if(!violated_starts_.empty())
      {
        int new_size = num_active_eq_;
        Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned> Aieq(mem_Aieq_.data(), m_robot.nv(), new_size);
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> bieq(mem_bieq_.data(), new_size);

        qp_.setMaxIter(100);
        qp_.reset(a_.size(), 0, Aieq.cols());
        eiquadprog::solvers::EiquadprogFast_status status =
          qp_.solve_quadprog(Q_, p_, Aeq_, beq_, -Aieq.transpose(), bieq, a_);

        if(status == eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL)
        {
          // everything is fine
          ROS_INFO_STREAM_THROTTLE(0.1, "fine");
        }
        else
        {
          // we have to tailor the constraints
          ROS_ERROR_STREAM("fail");

          int idx, cell;
          const auto& active = qp_.getActiveSet();
          for(size_t k = 0; k < qp_.getActiveSetSize(); ++k)
          {
            idx = active[k];
            cell = std::floor(idx / 4);
            
            // ROS_ERROR_STREAM("s=" << violated_starts_[cell] << " e=" << violated_sizes_[cell]);
            constraint_.upperBound().segment(violated_starts_[cell], violated_sizes_[cell]) //.setZero();
              = (a_zero_limits_.segment(violated_starts_[cell], violated_sizes_[cell]).array() < 0).select(0, a_zero_limits_.segment(violated_starts_[cell], violated_sizes_[cell]));
          }

        }
      }

      ROS_INFO_STREAM_THROTTLE(0.5, "TaskSkinDistanceConstraintHard: d_min=" << min_dist_);
      ROS_INFO_STREAM_THROTTLE(0.5, "TaskSkinDistanceConstraintHard: num_active_eq_=" << num_active_eq_);
      ROS_INFO_STREAM_THROTTLE(0.5, "TaskSkinDistanceConstraintHard: num_active_ieq_=" << num_active_ieq_);

      return constraint_;
    }    

    void TaskSkinDistanceConstraintHard::reconfigureRequest(Config &config, uint32_t level)
    {
      static bool is_first = true;
      if(is_first)
      {
        is_first = false;
        params_.toConfig(config);
      }
      else
      {
        params_.fromConfig(config);
      }
    }

    std::shared_ptr<TaskBase> TaskSkinDistanceConstraintHard::Load(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& name,
      bool activate,
      bool verbose)
    {
      cc::Parameters parameter(nh, name);
      parameter.addRequired<cc::Scalar>("dt");
      parameter.addRequired<cc::Scalar>("weight");
      if (!parameter.load())
      {
        ROS_ERROR("load_skin_distance_constraint_task: Failed to load parameters of '%s'", name.c_str());
        return nullptr;
      }
      cc::Scalar weight = parameter.get<cc::Scalar>("weight");
      
      auto task = std::make_shared<TaskSkinDistanceConstraintHard>(
        name, parameter.get<cc::Scalar>("dt"), tsid_wrapper.robotInterface().wrapper());
      if(!task->connect(nh))
      {
        ROS_ERROR("Failed to connect task='%s'", name.c_str());
        return nullptr;
      }
      if(verbose)
        ROS_INFO("Adding Task name='%s'", name.c_str());
      tsid_wrapper.loadTask(name, task, -1, 0, activate);
      return task;
    }
  }
}