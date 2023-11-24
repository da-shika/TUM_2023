#include <tsid/robots/robot-wrapper.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ics_tsid_tasks/skin/task_contact_distance_constraint_hard.h>

#include <control_core/ros/parameters.h>
#include <control_core/conversions.h>
#include <control_core/math/utilities.h>
#include <control_core/test_utilities/timing.h>

////////////////////////////////////////////////////////////////////////////////
// skin_client includes
////////////////////////////////////////////////////////////////////////////////
#include <skin_client/patch_client_factory.h>

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskContactDistanceConstraintHard::TaskContactDistanceConstraintHard(const std::string & name,
                                           cc::Scalar dt,
                                           const std::string & contact_topic,
                                           RobotWrapper & robot,
                                           tsid::InverseDynamicsFormulationAccForce & formulation):
      Base(name, robot),
      formulation_(formulation),
      dt_(dt),
      contact_topic_(contact_topic),
      constraint_(name)
    {
      //////////////////////////////////////////////////////////////////////////
      // setup members
      //////////////////////////////////////////////////////////////////////////
      T_f_w_.setIdentity();
      T_c_f_.setIdentity();
      T_c_j_.setIdentity();
      T_c_w_.setIdentity();
      J_.setZero(6, m_robot.nv());
      num_active_ieq_ = 0;
      num_violated_ieq_ = 0;
      min_distance_ = 1e10;

      //////////////////////////////////////////////////////////////////////////
      // setup inequality (set to default dim=1)
      //////////////////////////////////////////////////////////////////////////
      setMask(Vector::Ones(robot.na()));
      num_max_ieq_ = 1;
      constraint_.matrix().setZero(1, robot.nv());
      constraint_.lowerBound().setConstant(1, -1e10);
      constraint_.upperBound().setConstant(1, 1e10);

      //////////////////////////////////////////////////////////////////////////
      // setup fixed part of qp problem
      //////////////////////////////////////////////////////////////////////////
      int nv = robot.nv();
      
      Aeq_.resize(0, nv);
      beq_.resize(0, nv);
      Q_.setIdentity(nv,nv);
      p_.setZero(nv);
      
      // generator matrix
      G_ = cc::pointContactVRep(0.7).rowwise().normalized();
    }

    bool TaskContactDistanceConstraintHard::connect(ros::NodeHandle& nh)
    {
      //////////////////////////////////////////////////////////////////////////
      // load params
      //////////////////////////////////////////////////////////////////////////
      if(!params_.fromParamServer(nh, Base::name()))
      {
        ROS_ERROR("TaskContactDistanceConstraintHard:init(): Failed to init parameters");
        return false;
      }
      // setup parameters server
      reconfig_srv_ = std::make_unique<Server>(params_.privateNamespace());
      reconfig_srv_->setCallback(boost::bind(&TaskContactDistanceConstraintHard::reconfigureRequest, this, _1, _2));

      //////////////////////////////////////////////////////////////////////////
      // setup contacts
      //////////////////////////////////////////////////////////////////////////
      auto contact_frames = skin_client::list_available_frames();
      for(const auto& frame : contact_frames)
      {
        if(m_robot.model().existFrame(frame))
        {
          Index id = m_robot.model().getFrameId(frame);
          frame_ids_[frame] = id;
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // setup inequality (set to dim=1 in case no patch could be loaded)
      //////////////////////////////////////////////////////////////////////////
      num_max_ieq_ = 16; //3*std::max(size_t(1), frame_ids_.size());                  // TODO: lets say 3 per link for now
      constraint_.matrix().setZero(num_max_ieq_, Base::m_robot.nv());
      constraint_.lowerBound().setConstant(num_max_ieq_, -1e10);
      constraint_.upperBound().setConstant(num_max_ieq_, 1e10);
      a_zero_limits_.setZero(num_max_ieq_);

      //////////////////////////////////////////////////////////////////////////
      // setup matrix buffers
      //////////////////////////////////////////////////////////////////////////
      int rows = 4*num_max_ieq_;
      int cols = Base::m_robot.nv();
      mem_Aieq_.resize(rows*cols, 0.0);
      mem_bieq_.resize(rows, 0.0);

      //////////////////////////////////////////////////////////////////////////
      // setup connection to wbc generator
      //////////////////////////////////////////////////////////////////////////
      if(!cc::is_topic_published(contact_topic_, "control_core_msgs::SkinPatches"))
      {
        ROS_ERROR("TaskContactDistanceConstraintHard::connect: Topic '%s' not yet published", contact_topic_.c_str());
        //return false;
      }
      wbc_contacts_sub_ = nh.subscribe(contact_topic_, 1, &TaskContactDistanceConstraintHard::callback, this);

      //////////////////////////////////////////////////////////////////////////
      // debug msg
      //////////////////////////////////////////////////////////////////////////
      msg_pub_ = nh.advertise<ics_tsid_task_msgs::SkinDistanceConstraint>(Base::name() + "_debug", 1);

      ROS_INFO_STREAM("TaskContactDistanceConstraintHard::connect(): max contacts=" << num_max_ieq_);
      return true;
    }

    void TaskContactDistanceConstraintHard::setMask(ConstRefVector m)
    {
      assert(m.size()==m_robot.na());
      m_mask = m;
    }

    double TaskContactDistanceConstraintHard::minimumDistance() const
    {
      return min_distance_;
    }

    int TaskContactDistanceConstraintHard::numActiveConstraints() const
    {
      return num_active_ieq_;
    }

    const ConstraintBase & TaskContactDistanceConstraintHard::compute(const double t,
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

      // reset all constraints
      constraint_.upperBound().setConstant(1e10);

      // reset msg
      msg_.distances.data.clear();
      msg_.vel_limits.data.clear();
      msg_.vel_cmds.data.clear();
      msg_.acc_limits.data.clear();
      msg_.cell_nums.data.clear();

      active_frame_ids_.clear();
      violated_frame_ids_.clear();
      violated_indices_.clear();

      //////////////////////////////////////////////////////////////////////////
      // iterate over all contacts
      //////////////////////////////////////////////////////////////////////////
      size_t k = 0;
      auto start_time = TIMENOW();

      for(size_t i = 0; i < std::min(contacts_.size(), num_max_ieq_); ++i)
      {
        const auto& contact = contacts_[i];
        double d = contact.minDist();
        double f = contact.force().W().z();

        if(d < params_.d_start)
        {
          // store some information
          if(f > max_force_)
            max_force_ = f;
          if(d < min_distance_) {
            min_distance_ = d;
          }

          // get the joint frame and corresponding chain
          Index frame_id = frame_ids_[contact.frame()];
          const Frame & frame = m_robot.model().frames[frame_id];
          const JointIndex joint_id = frame.parent;
          active_frame_ids_.push_back(frame_id);

          // frame placement
          T_f_w_ = data.oMi[joint_id] * frame.placement;

          // contact placement of contact wrt frame
          T_c_f_.translation(contact.pose().linear());
          T_c_f_.rotation(contact.pose().angular().toRotationMatrix());

          // contact wrt world
          T_c_w_ = T_f_w_ * T_c_f_;
          // contact wrt joint
          T_c_j_ = frame.placement * T_c_f_;

          // build the jacobain to the contact
          J_.setZero();
          pinocchio::details::translateJointJacobian(
            m_robot.model(), data, joint_id, pinocchio::LOCAL, T_c_w_, data.J, J_);

          // collision space of contact is z axis
          constraint_.matrix().row(k) = J_.middleRows(2,1);

          // bias acceleration in cell frame
          drift_ = T_c_j_.actInv(data.a[joint_id]);
          v_joint_ = T_c_j_.actInv(data.v[joint_id]);
          drift_.linear() += v_joint_.angular().cross(v_joint_.linear());

          // dP > 0 moving into the collision, dP < 0 backing off
          double dP = constraint_.matrix().row(k).dot(v);

          // acceleration limit
          cc::Scalar fac = (d - params_.d_min) / (params_.d_start - params_.d_min);
          cc::Scalar vel_limit = std::max(-std::abs(params_.v_max), fac*params_.v_damp);
          cc::Scalar acc_limit = (vel_limit - dP) / dt_;

          a_zero_limits_[k] = (0.0 - dP) / dt_ - drift_.linear().z();

          // check how to handle this action
          if(d > params_.d_min + 0.005)
          {
            // this is not violated, limit velocity
            constraint_.upperBound()[k] = acc_limit - drift_.linear().z();
            num_active_ieq_++;
          }
          else
          {
            // this is violated, set velocity to zero
            constraint_.upperBound()[k] = acc_limit - drift_.linear().z();

            // resize test qp
            violated_frame_ids_.push_back(frame_id);
            violated_indices_.push_back(k);

            int new_size = num_violated_ieq_ + 4;
            Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned> Aieq(mem_Aieq_.data(), m_robot.nv(), new_size);
            Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> bieq(mem_bieq_.data(), new_size);

            Aieq.rightCols(4) = (G_*J_.topRows(3)).transpose();
            bieq.tail(4).setConstant(-1); // constraint_.upperBound()[k]    // no need for real velocity!
            num_violated_ieq_ += 4;
          }

        }
        k++;
      }

      if(num_violated_ieq_ > 0)
      {
        // we have violated constraints and need to check if they are feasible

        Eigen::Map<Eigen::MatrixXd, Eigen::Unaligned> Aieq(mem_Aieq_.data(), m_robot.nv(), num_violated_ieq_);
        Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> bieq(mem_bieq_.data(), num_violated_ieq_);

        // solve qp: 
        int nv = m_robot.nv();
        int n_ieq = Aieq.cols();
        int n_prime = nv + n_ieq;

        Q_prime_.setIdentity(n_prime, n_prime);
        Q_prime_.bottomRightCorner(n_ieq, n_ieq) *= 1e8;
        p_prime_.setZero(n_prime);

        Aieq_prime_.resize(2*n_ieq, n_prime);
        Aieq_prime_.block(0, 0, n_ieq, nv) = Aieq.transpose();
        Aieq_prime_.block(0, nv, n_ieq, n_ieq) = -cc::MatrixX::Identity(n_ieq, n_ieq);
        Aieq_prime_.block(n_ieq, 0, n_ieq, nv).setZero();
        Aieq_prime_.block(n_ieq, nv, n_ieq, n_ieq) = -cc::MatrixX::Identity(n_ieq, n_ieq);
        
        bieq_prime_.resize(2*n_ieq);
        bieq_prime_.head(n_ieq) = bieq;
        bieq_prime_.tail(n_ieq).setZero();

        qp_.reset(n_prime, 0, 2*n_ieq);
        eiquadprog::solvers::EiquadprogFast_status status =
          qp_.solve_quadprog(Q_prime_, p_prime_, Aeq_, beq_, -Aieq_prime_, bieq_prime_, sol_prime_);
        
        int k, frame_id;
        for(size_t l = 0; l < n_ieq; ++l)
        {
          if(sol_prime_[nv+l] > 1e-8)
          {
            // this constraint is violated
            frame_id = violated_frame_ids_[std::floor(l / 4)];

            // check all the others
            for(size_t m = 0; m < violated_frame_ids_.size(); ++m)
            {
              if(frame_id == violated_frame_ids_[m])
              {
                k = violated_indices_[m];
                constraint_.upperBound()[k] = std::max(a_zero_limits_[k], 0.0);
              }
            }
          }
        }
      }

      //////////////////////////////////////////////////////////////////////////
      // publish debugging information
      //////////////////////////////////////////////////////////////////////////

      msg_.dur.data = DURATION(start_time);
      msg_.num_active_ieq.data = num_active_ieq_;
      msg_.num_violated_ieq.data = num_violated_ieq_;
      msg_.max_force.data = max_force_;
      msg_.min_distance.data = min_distance_;
      msg_.max_proximity.data = max_proximity_;
      cc::publish_if_subscribed(msg_pub_, msg_);

      ROS_WARN_STREAM_THROTTLE(0.5, "TaskContactDistanceConstraintHard: d_min=" << min_distance_);
      ROS_WARN_STREAM_THROTTLE(0.5, "TaskContactDistanceConstraintHard: num_active_ieq=" << num_active_ieq_);
      ROS_WARN_STREAM_THROTTLE(0.5, "TaskContactDistanceConstraintHard: num_violated_ieq=" << num_violated_ieq_);

      return constraint_;
    }    

    void TaskContactDistanceConstraintHard::reconfigureRequest(Config &config, uint32_t level)
    {
      params_.updateConfig(config, level);
    }

    void TaskContactDistanceConstraintHard::callback(const control_core_msgs::SkinPatchesConstPtr& msg)
    {
      contacts_.resize(msg->patches.size());
      for(size_t i = 0; i < contacts_.size(); ++i)
        contacts_[i] = msg->patches[i];
    }

    std::shared_ptr<TaskBase> TaskContactDistanceConstraintHard::Load(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& name,
      bool activate,
      bool verbose)
    {
      cc::Parameters parameter(nh, name);
      parameter.addRequired<cc::Scalar>("dt");
      parameter.addRequired<std::string>("contact_topic");
      parameter.addRequired<cc::VectorX>("kp");
      if (!parameter.load())
      {
        ROS_ERROR("load_skin_distance_constraint_task: Failed to load parameters of '%s'", name.c_str());
        return nullptr;
      }

      auto task = std::make_shared<TaskContactDistanceConstraintHard>(
        name, parameter.get<cc::Scalar>("dt"),
        parameter.get<std::string>("contact_topic"),
        tsid_wrapper.robotInterface().wrapper(), tsid_wrapper.formulation());
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
