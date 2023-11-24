#include <tsid/robots/robot-wrapper.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ics_tsid_tasks/skin/task_contact_distance_constraint_soft.h>

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

    TaskContactDistanceConstraintSoft::TaskContactDistanceConstraintSoft(const std::string & name,
                                           cc::Scalar dt,
                                           cc::Scalar max_weight,
                                           const std::string & contact_topic,
                                           RobotWrapper & robot,
                                           tsid::InverseDynamicsFormulationAccForce & formulation,
                                           MotionTask motion_task):
      Base(name, robot),
      formulation_(formulation),
      dt_(dt),
      contact_topic_(contact_topic),
      max_weight_(max_weight),
      constraint_(name),
      motion_task_(motion_task)
    {
      //////////////////////////////////////////////////////////////////////////
      // setup members
      //////////////////////////////////////////////////////////////////////////
      T_c_f_.setIdentity();
      T_c_j_.setIdentity();
      T_f_w_.setIdentity();
      J_.setZero(6, m_robot.nv());
      num_active_ieq_ = 0;
      num_violated_ieq_ = 0;
      min_distance_ = 1e10;

      t_release_ = 0.0;
      weight_ = 0.0;

      //////////////////////////////////////////////////////////////////////////
      // setup inequality (set to default dim=1)
      //////////////////////////////////////////////////////////////////////////
      setMask(Vector::Ones(robot.na()));
      num_max_ieq_ = 1;
      constraint_.matrix().setZero(1, robot.nv());
      constraint_.lowerBound().setConstant(1, -1e10);
      constraint_.upperBound().setConstant(1, 1e10);
    }

    bool TaskContactDistanceConstraintSoft::connect(ros::NodeHandle& nh)
    {
      //////////////////////////////////////////////////////////////////////////
      // load params
      //////////////////////////////////////////////////////////////////////////
      if(!params_.fromParamServer(nh, Base::name()))
      {
        ROS_ERROR("TaskContactDistanceConstraintSoft:init(): Failed to init parameters");
        return false;
      }
      // setup parameters server
      reconfig_srv_ = std::make_unique<Server>(params_.privateNamespace());
      reconfig_srv_->setCallback(boost::bind(&TaskContactDistanceConstraintSoft::reconfigureRequest, this, _1, _2));

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
      num_max_ieq_ = 20; //3*std::max(size_t(1), frame_ids_.size());                  // TODO: lets say 3 per link for now
      constraint_.matrix().setZero(num_max_ieq_, Base::m_robot.nv());
      constraint_.lowerBound().setConstant(num_max_ieq_, -1e10);
      constraint_.upperBound().setConstant(num_max_ieq_, 1e10);

      //////////////////////////////////////////////////////////////////////////
      // setup connection to wbc generator
      //////////////////////////////////////////////////////////////////////////
      if(!cc::is_topic_published(contact_topic_, "control_core_msgs::SkinPatches"))
      {
        ROS_ERROR("TaskContactDistanceConstraintSoft::connect: Topic '%s' not yet published", contact_topic_.c_str());
        //return false;
      }
      wbc_contacts_sub_ = nh.subscribe(contact_topic_, 1, &TaskContactDistanceConstraintSoft::callback, this);

      //////////////////////////////////////////////////////////////////////////
      // debug msg
      //////////////////////////////////////////////////////////////////////////
      msg_pub_ = nh.advertise<ics_tsid_task_msgs::SkinDistanceConstraint>(Base::name() + "_debug", 1);

      ROS_INFO_STREAM("TaskContactDistanceConstraintSoft::connect(): max contacts=" << num_max_ieq_);
      return true;
    }

    void TaskContactDistanceConstraintSoft::setMask(ConstRefVector m)
    {
      assert(m.size()==m_robot.na());
      m_mask = m;
    }

    double TaskContactDistanceConstraintSoft::minimumDistance() const
    {
      return min_distance_;
    }

    int TaskContactDistanceConstraintSoft::numActiveConstraints() const
    {
      return num_active_ieq_;
    }

    const ConstraintBase & TaskContactDistanceConstraintSoft::compute(const double t,
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

      // reset all constraints
      constraint_.upperBound().setConstant(1e10);

      // reset msg
      msg_.distances.data.clear();
      msg_.vel_limits.data.clear();
      msg_.vel_cmds.data.clear();
      msg_.acc_limits.data.clear();
      msg_.cell_nums.data.clear();
      
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

          // check how to handle this action
          if(d > params_.d_min)
          {
            // this is not violated, limit velocity
            constraint_.upperBound()[k] = acc_limit - drift_.linear().z();
            num_active_ieq_++;
          }
          else
          {
            // this is violated, set velocity to zero
            cc::Scalar acc_zero_limit = (0.0 - dP) / dt_;
            constraint_.upperBound()[k] = acc_zero_limit - drift_.linear().z();

            // and add violation acc to relaxed task
            const auto& J = constraint_.matrix().row(k).rightCols(m_robot.na());
            a_relaxed += J.transpose()/J.norm() * (acc_limit - drift_.linear().z());
            num_violated_ieq_++;
          }

        }
        k++;
      }

      if(num_violated_ieq_)
      {
        // scale the weight exponentially based on minimum distance
        cc::Scalar fac = 1.0 - cc::clamp(min_distance_ / params_.d_min, 0.0, 1.0);
        weight_ = max_weight_ * fac;
        t_release_ = t;
        //a_relaxed /= num_violated_ieq_;
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

      // ROS_INFO_STREAM_THROTTLE(0.5, "TaskContactDistanceConstraintSoft: d_min=" << min_distance_);
      // ROS_INFO_STREAM_THROTTLE(0.5, "TaskContactDistanceConstraintSoft: num_active_ieq=" << num_active_ieq_);
      // ROS_INFO_STREAM_THROTTLE(0.5, "TaskContactDistanceConstraintSoft: num_violated_ieq=" << num_violated_ieq_);

      return constraint_;
    }    

    void TaskContactDistanceConstraintSoft::reconfigureRequest(Config &config, uint32_t level)
    {
      params_.updateConfig(config, level);
      motion_task_->setKp(params_.kp);
    }

    void TaskContactDistanceConstraintSoft::callback(const control_core_msgs::SkinPatchesConstPtr& msg)
    {
      contacts_.resize(msg->patches.size());
      for(size_t i = 0; i < contacts_.size(); ++i)
        contacts_[i] = msg->patches[i];
    }

    std::shared_ptr<TaskBase> TaskContactDistanceConstraintSoft::Load(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& name,
      bool activate,
      bool verbose)
    {
      cc::Parameters parameter(nh, name);
      parameter.addRequired<cc::Scalar>("dt");
      parameter.addRequired<std::string>("contact_topic");
      parameter.addRequired<cc::Scalar>("weight");
      parameter.addRequired<cc::VectorX>("kp");
      if (!parameter.load())
      {
        ROS_ERROR("load_skin_distance_constraint_task: Failed to load parameters of '%s'", name.c_str());
        return nullptr;
      }
      cc::Scalar weight = parameter.get<cc::Scalar>("weight");

      // repulive force motion task
      auto motion_task = std::make_shared<TaskContactDistanceConstraintMotion>(
        name+"_relaxed", tsid_wrapper.robotInterface().wrapper());
      motion_task->setKp(parameter.get<cc::VectorX>("kp"));
      motion_task->setKd(2.0*motion_task->Kp().cwiseSqrt());
      
      auto task = std::make_shared<TaskContactDistanceConstraintSoft>(
        name, parameter.get<cc::Scalar>("dt"), weight,
        parameter.get<std::string>("contact_topic"),
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
// TaskContactDistanceConstraintMotion
////////////////////////////////////////////////////////////////////////////////

    TaskContactDistanceConstraintMotion::TaskContactDistanceConstraintMotion(const std::string & name, RobotWrapper & robot) :
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

    void TaskContactDistanceConstraintMotion::setMask(ConstRefVector m)
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

    void TaskContactDistanceConstraintMotion::setKp(const Vector& Kp)
    {
      assert(Kp.size()==m_robot.na());
      Kp_ = Kp;
    }

    void TaskContactDistanceConstraintMotion::setKd(const Vector& Kd)
    {
      assert(Kd.size()==m_robot.na());
      Kd_ = Kd;
    }

    const ConstraintBase & TaskContactDistanceConstraintMotion::compute(const double ,
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