#include <tsid/robots/robot-wrapper.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ics_tsid_tasks/skin/task_skin_distance_constraint_soft.h>

#include <control_core/ros/parameters.h>
#include <control_core/conversions.h>
#include <control_core/math/utilities.h>
#include <control_core/test_utilities/timing.h>

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskSkinDistanceConstraintSoft::TaskSkinDistanceConstraintSoft(const std::string & name,
                                           cc::Scalar dt,
                                           cc::Scalar max_weight,
                                           RobotWrapper & robot,
                                           tsid::InverseDynamicsFormulationAccForce & formulation,
                                           MotionTask motion_task):
      Base(name, robot),
      formulation_(formulation),
      dt_(dt),
      max_weight_(max_weight),
      constraint_(name),
      motion_task_(motion_task)
    {
      //////////////////////////////////////////////////////////////////////////
      // setup members
      //////////////////////////////////////////////////////////////////////////
      X_c_b.setIdentity();
      placement.setIdentity();
      J_cell.setZero(6, m_robot.nv());
      num_active_ieq_ = 0;
      num_violated_ieq_ = 0;
      min_distance_ = 1e10;

      t_release_ = 0.0;
      weight_ = 0.0;

      //////////////////////////////////////////////////////////////////////////
      // setup inequality (set to default dim=1)
      //////////////////////////////////////////////////////////////////////////
      setMask(Vector::Ones(robot.na()));
      constraint_.matrix().setZero(1, robot.nv());
      constraint_.lowerBound().setConstant(1, -1e10);
      constraint_.upperBound().setConstant(1, 1e10);
      distances_lp_.resize(1, params_.d_start + 0.02);
    }

    bool TaskSkinDistanceConstraintSoft::connect(ros::NodeHandle& nh)
    {
      //////////////////////////////////////////////////////////////////////////
      // load params
      //////////////////////////////////////////////////////////////////////////
      if(!params_.fromParamServer(nh, Base::name()))
      {
        ROS_ERROR("TaskSkinDistanceConstraintSoft:init(): Failed to init parameters");
        return false;
      }
      // setup parameters server
      reconfig_srv_ = std::make_unique<Server>(params_.privateNamespace());
      reconfig_srv_->setCallback(boost::bind(&TaskSkinDistanceConstraintSoft::reconfigureRequest, this, _1, _2));

      //////////////////////////////////////////////////////////////////////////
      // setup clients
      //////////////////////////////////////////////////////////////////////////
      size_t total_number_cells;
      tsid::tasks::TaskSkinDistanceConstraintSoft::Clients client_candidates;
      if(!skin_client::load_clients(client_candidates, total_number_cells))
      {
        ROS_WARN("TaskSkinDistanceConstraintSoft:init() '%s' could not find any servers", Base::name().c_str());
      }

      //////////////////////////////////////////////////////////////////////////
      // check if frame is known, map frame id to jacobian
      //////////////////////////////////////////////////////////////////////////
      total_number_cells = 0;
      for(auto client : client_candidates)
      {
        if(cc::has(params_.masked_patches, client->name()))
        {
          ROS_WARN("TaskSkinDistanceConstraintSoft:init() '%s' masked",client->name().c_str()); 
          continue;
        }
        if(m_robot.model().existFrame(client->jointFrame()))
        {
          Index id = m_robot.model().getFrameId(client->jointFrame());
          client_ids_.push_back(id);
          clients_.push_back(client);

          client_start_idx_.push_back(total_number_cells);
          total_number_cells += client->numberOfCells();
          client_end_idx_.push_back(total_number_cells);
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // setup inequality (set to dim=1 in case no patch could be loaded)
      //////////////////////////////////////////////////////////////////////////
      total_number_cells = std::max(size_t(1), total_number_cells);
      constraint_.matrix().setZero(total_number_cells, Base::m_robot.nv());
      constraint_.lowerBound().setConstant(total_number_cells, -1e10);
      constraint_.upperBound().setConstant(total_number_cells, 1e10);
      distances_lp_.resize(total_number_cells, params_.d_start + 0.02);

      //////////////////////////////////////////////////////////////////////////
      // connect them
      //////////////////////////////////////////////////////////////////////////
      ros::NodeHandle internal_nh("~");
      for(auto client : clients_)
        client->enableDataConnection(internal_nh);

      //////////////////////////////////////////////////////////////////////////
      // debug msg
      //////////////////////////////////////////////////////////////////////////
      msg_pub_ = nh.advertise<ics_tsid_task_msgs::SkinDistanceConstraint>(Base::name() + "_debug", 1);

      ROS_INFO_STREAM("TaskSkinDistanceConstraintSoft::TaskSkinDistanceConstraintSoft(): cells=" << 
        total_number_cells << " clients=" << clients_.size());
      for(size_t i = 0; i < clients_.size(); ++i)
        ROS_INFO("client: '%s' idx_range=(%ld,%ld)", 
          clients_[i]->name().c_str(), client_start_idx_[i], client_end_idx_[i]);

      return true;
    }

    void TaskSkinDistanceConstraintSoft::setMask(ConstRefVector m)
    {
      assert(m.size()==m_robot.na());
      m_mask = m;
    }

    double TaskSkinDistanceConstraintSoft::minimumDistance() const
    {
      return min_distance_;
    }

    int TaskSkinDistanceConstraintSoft::numActiveConstraints() const
    {
      return num_active_ieq_;
    }

    const ConstraintBase & TaskSkinDistanceConstraintSoft::compute(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
    {
      typedef typename Matrix6x::ColXpr ColXprIn;
      typedef const pinocchio::MotionRef<ColXprIn> MotionIn;
      typedef typename Matrix6x::ColXpr ColXprOut;
      typedef MotionRef<ColXprOut> MotionOut;

      // reset states
      min_distance_ = 1e10;
      max_force_ = 0.0;
      max_proximity_ = 0.0;
      num_active_ieq_ = 0;
      num_violated_ieq_ = 0;

      // reset relaxed acc
      auto& a_relaxed = motion_task_->desAcceleration();
      a_relaxed.setZero();

      // reset msg
      msg_.distances.data.clear();
      msg_.vel_limits.data.clear();
      msg_.vel_cmds.data.clear();
      msg_.acc_limits.data.clear();
      msg_.cell_nums.data.clear();
      
      //////////////////////////////////////////////////////////////////////////
      // iterate over all patches and update the constraints
      //////////////////////////////////////////////////////////////////////////
      bool valid = false;
      size_t k = 0;
      auto start_time = TIMENOW();

      for(size_t i = 0; i < clients_.size(); ++i)
      {
        // patch data
        Client client = clients_[i];
        const auto& points = client->data().positions;
        const auto& normals = client->data().normals;
        const auto& distances = client->data().distance;
        const auto& proximity = client->data().proximity;
        const auto& forces = client->data().force;   
        const auto& neighbors = client->data().neighbors; 
  
        // get the joint frame and corresponding chain
        Index frame_id = client_ids_[i];
        const Frame & frame = m_robot.model().frames[frame_id];
        const JointIndex joint_id = frame.parent;
        
        // frame placement
        typename Data::SE3 & oMframe = data.oMf[frame_id];
        oMframe = data.oMi[joint_id] * frame.placement;

        ////////////////////////////////////////////////////////////////////////
        // update cells
        ////////////////////////////////////////////////////////////////////////
        for(size_t j = 0; j < client->numberOfCells(); ++j)
        {
          const auto& n = normals.col(j);
          const auto& p = points.col(j);
          const auto& neigbors_j = neighbors[j];
          double d = distances[j];
          double f = forces[j];
          double prox = proximity[j];

          // lowpass filter this signal
          distances_lp_[k] = params_.alpha*distances_lp_[k] + (1.-params_.alpha)*d;
          double d_lp = distances_lp_[k];

          // check if this cell is valid based on neighbors
          valid = false;
          if(d_lp < params_.d_start)
          {
            for(auto l : neigbors_j) {
              if(distances[l] < params_.d_start+0.01) {
                valid = true; break;
              }
            }
          }
        
          // this constraint has to do something
          if(valid)
          {
            // store some information
            if(f > max_force_)
              max_force_ = f;
            if(d_lp < min_distance_) {
              min_distance_ = d_lp;
              max_proximity_ = prox;
            }

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
            cc::Scalar fac = (d_lp - params_.d_min) / (params_.d_start - params_.d_min);
            cc::Scalar vel_limit = std::max(-std::abs(params_.v_max), fac*params_.v_damp);
            cc::Scalar acc_limit = (vel_limit - dP) / dt_;

            // check how to handle this action
            if(d_lp > params_.d_min)
            {
              // this is not violated, limit velocity
              constraint_.upperBound()[k] = acc_limit - n.dot(drift_b.linear());
              num_active_ieq_++;
            }
            else
            {
              // this is violated, set velocity to zero
              cc::Scalar acc_zero_limit = (0.0 - dP) / dt_;
              constraint_.upperBound()[k] = acc_zero_limit - n.dot(drift_b.linear());

              // and add violation acc to relaxed task
              const auto& J_c = constraint_.matrix().row(k).rightCols(m_robot.na());
              a_relaxed += J_c.transpose()/J_c.norm() * (acc_limit - n.dot(drift_b.linear()));
              num_violated_ieq_++;
            }

            // set msg debugging information
            msg_.cell_nums.data.push_back(k);
            msg_.distances.data.push_back(d_lp);
            msg_.vel_limits.data.push_back(vel_limit);
            msg_.vel_cmds.data.push_back(dP);
            msg_.acc_limits.data.push_back(constraint_.upperBound()[k]);
          }
          else
          {
            // deactivate
            constraint_.upperBound()[k] = 1e10;
          }

          k++;
        }
      }

      if(num_violated_ieq_)
      {
        cc::Scalar fac = 1.0 - cc::clamp(min_distance_ / params_.d_min, 0.0, 1.0);
        weight_ = max_weight_ * fac;
        t_release_ = t;
        a_relaxed /= num_violated_ieq_;
      }
      formulation_.updateTaskWeight(motion_task_->name(), weight_*std::exp(-10.0*(t - t_release_)));

      //////////////////////////////////////////////////////////////////////////
      // publish debugging information
      //////////////////////////////////////////////////////////////////////////

      msg_.dur.data = DURATION(start_time);
      msg_.weight.data = weight_;
      msg_.num_active_ieq.data = num_active_ieq_;
      msg_.num_violated_ieq.data = num_violated_ieq_;
      msg_.max_force.data = max_force_;
      msg_.min_distance.data = min_distance_;
      msg_.max_proximity.data = max_proximity_;
      control_core::eigenToStdVector(motion_task_->acceleration(), msg_.a_relaxed.data);
      cc::publish_if_subscribed(msg_pub_, msg_);

      // ROS_INFO_STREAM_THROTTLE(0.5, "TaskSkinDistanceConstraintSoft: d_min=" << min_distance_);
      // ROS_INFO_STREAM_THROTTLE(0.5, "TaskSkinDistanceConstraintSoft: num_active_ieq=" << num_active_ieq_);
      // ROS_INFO_STREAM_THROTTLE(0.5, "TaskSkinDistanceConstraintSoft: num_violated_ieq=" << num_violated_ieq_);

      return constraint_;
    }    

    void TaskSkinDistanceConstraintSoft::reconfigureRequest(Config &config, uint32_t level)
    {
      params_.updateConfig(config, level);
      motion_task_->setKp(params_.kp);
    }

    std::shared_ptr<TaskBase> TaskSkinDistanceConstraintSoft::Load(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& name,
      bool activate,
      bool verbose)
    {
      cc::Parameters parameter(nh, name);
      parameter.addRequired<cc::Scalar>("dt");
      parameter.addRequired<cc::Scalar>("weight");
      parameter.addRequired<cc::VectorX>("kp");
      if (!parameter.load())
      {
        ROS_ERROR("load_skin_distance_constraint_task: Failed to load parameters of '%s'", name.c_str());
        return nullptr;
      }
      cc::Scalar weight = parameter.get<cc::Scalar>("weight");

      // repulive force motion task
      auto motion_task = std::make_shared<TaskSkinDistanceMotion>(
        name+"_relaxed", tsid_wrapper.robotInterface().wrapper());
      motion_task->setKp(parameter.get<cc::VectorX>("kp"));
      motion_task->setKd(2.0*motion_task->Kp().cwiseSqrt());

      auto task = std::make_shared<TaskSkinDistanceConstraintSoft>(
        name, parameter.get<cc::Scalar>("dt"), weight,
        tsid_wrapper.robotInterface().wrapper(), tsid_wrapper.formulation(), motion_task);
      if(!task->connect(nh))
      {
        ROS_ERROR("Failed to connect task='%s'", name.c_str());
        return nullptr;
      }
      if(verbose)
        ROS_INFO("Adding Task name='%s'", name.c_str());
      tsid_wrapper.loadTask(name+"_relaxed", motion_task, 0.0, 1, activate);    
      tsid_wrapper.loadTask(name, task, -1, 0, activate);
      return task;
    }

////////////////////////////////////////////////////////////////////////////////
// TaskSkinDistanceMotion
////////////////////////////////////////////////////////////////////////////////

    TaskSkinDistanceMotion::TaskSkinDistanceMotion(const std::string & name, RobotWrapper & robot) :
      Base(name, robot),
      constraint_(name, robot.na(), robot.nv())
    {
      a_.setZero(robot.na());
      a_des_.setZero(robot.na());
      Kp_.setZero(robot.na());
      Kd_.setZero(robot.na());
      Vector m = Vector::Ones(robot.na());
      setMask(m);
    }

    void TaskSkinDistanceMotion::setMask(ConstRefVector m)
    {
      assert(m.size()==m_robot.na());
      m_mask = m;
      const Vector::Index dim = static_cast<Vector::Index>(m.sum());
      Matrix S = Matrix::Zero(dim, m_robot.nv());
      active_axes_.resize(dim);
      unsigned int j=0;
      for(unsigned int i=0; i<m.size(); i++)
        if(m(i)!=0.0)
        {
          assert(m(i)==1.0);
          S(j,m_robot.nv()-m_robot.na()+i) = 1.0;
          active_axes_(j) = i;
          j++;
        }
      constraint_.resize((unsigned int)dim, m_robot.nv());
      constraint_.setMatrix(S);
      constraint_.vector().setZero();
    }

    void TaskSkinDistanceMotion::setKp(const Vector& Kp)
    {
      assert(Kp.size()==m_robot.na());
      Kp_ = Kp;
    }

    void TaskSkinDistanceMotion::setKd(const Vector& Kd)
    {
      assert(Kd.size()==m_robot.na());
      Kd_ = Kd;
    }

    const ConstraintBase & TaskSkinDistanceMotion::compute(const double ,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
    {
      a_ = Kp_.cwiseProduct(a_des_) - Kd_.cwiseProduct(v.tail(m_robot.na()));
      for(unsigned int i=0; i<active_axes_.size(); i++)
        constraint_.vector()(i) = a_(active_axes_(i));
      return constraint_;
    }

  }
}