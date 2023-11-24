#include <tsid/robots/robot-wrapper.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ics_tsid_tasks/skin/task_skin_joint_compliance.h>

#include <control_core/ros/parameters.h>

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskSkinJointCompliance::TaskSkinJointCompliance(const std::string & name,
                                           RobotWrapper & robot,
                                           const std::string& skinTopicName):
      Base(name, robot),
      skin_topic_name_(skinTopicName),
      constraint_(name, robot.na(), robot.nv())
    {
      tau_ext_.setZero(robot.na());
      W_des_.setZero();
      Kp_.setZero(robot.na());
      Kd_.setZero(robot.na());

      J_c_.setZero(6, robot.nv());
      T_c_w_.setIdentity();
      T_c_f_.setIdentity();
      T_c_w_.setIdentity();

      Vector m = Vector::Ones(robot.na());
      setMask(m);

      has_patches_callback_ = false;
    }

    void TaskSkinJointCompliance::setMask(ConstRefVector m)
    {
      assert(m.size()==m_robot.na());
      m_mask = m;
      const Vector::Index dim = static_cast<Vector::Index>(m.sum());
      Matrix S = Matrix::Zero(dim, m_robot.nv());
      active_axis_.resize(dim);
      unsigned int j=0;
      for(unsigned int i=0; i<m.size(); i++)
        if(m(i)!=0.0)
        {
          assert(m(i)==1.0);
          S(j,m_robot.nv()-m_robot.na()+i) = 1.0;
          active_axis_(j) = i;
          j++;
        }
      constraint_.resize((unsigned int)dim, m_robot.nv());
      constraint_.setMatrix(S);
    }

    void TaskSkinJointCompliance::setAlphaProximity(double alphaProximity)
    {
      alpha_proximity_ = std::min(1.0, std::max(0.0, alphaProximity));
    }

    int TaskSkinJointCompliance::dim() const
    {
      return (int)m_mask.sum();
    }

    bool TaskSkinJointCompliance::subscribe(ros::NodeHandle& nh)
    {
      // setup ros topic
      bool ret = true;
      if(!cc::is_topic_published(skin_topic_name_, "control_core_msgs::SkinData"))
      {
        ROS_ERROR("TaskSkinJointCompliance::subscribe: Topic '%s' not yet published", skin_topic_name_.c_str());
        ret = false;
      }
      contacts_sub_ = nh.subscribe(skin_topic_name_, 1, &TaskSkinJointCompliance::callback, this);
      
      // load parameters
      if(!params_.fromParamServer(nh, Base::name()))
      {
        ROS_ERROR("TaskSkinJointCompliance:init(): Failed to init parameters");
        return false;
      }
      // setup parameters server
      reconfig_srv_ = std::make_unique<Server>(params_.privateNamespace());
      reconfig_srv_->setCallback(boost::bind(&TaskSkinJointCompliance::reconfigureRequest, this, _1, _2));

      return ret;
    }

    const Vector & TaskSkinJointCompliance::Kp(){ return Kp_; }

    const Vector & TaskSkinJointCompliance::Kd(){ return Kd_; }

    void TaskSkinJointCompliance::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==m_robot.na());
      Kp_ = Kp;
    }

    void TaskSkinJointCompliance::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==m_robot.na());
      Kd_ = Kd;
    }

    const Vector & TaskSkinJointCompliance::getDesiredAcceleration() const
    {
      return a_des_;
    }

    Vector TaskSkinJointCompliance::getAcceleration(ConstRefVector dv) const
    {
      return constraint_.matrix()*dv;
    }

    const Vector & TaskSkinJointCompliance::getTauExt() const
    {
      return tau_ext_;
    }

    const ConstraintBase & TaskSkinJointCompliance::getConstraint() const
    {
      return constraint_;
    }

    const ConstraintBase & TaskSkinJointCompliance::compute(const double ,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
    {
      // compute the compliance torque of actuated joints
      Vector W;
      tau_ext_.setZero();
      for(const auto& contact : contacts_)
      {
        W = (1.0 - alpha_proximity_)*contact.force().wrench() 
              + alpha_proximity_*contact.proximity().wrench();

        ROS_INFO_STREAM_THROTTLE(1.0, "alpha_proximity_=" << alpha_proximity_);
        ROS_INFO_STREAM_THROTTLE(1.0, "W_force=" << contact.force().wrench().z());
        ROS_INFO_STREAM_THROTTLE(1.0, "W_prox=" << contact.proximity().wrench().z());
        ROS_INFO_STREAM_THROTTLE(1.0, "W=" << W.z());

        if(computeContactJacobian(contact, data, J_c_))
        {
          tau_ext_ += J_c_.rightCols(m_robot.na()).transpose() * (W_des_ - W);
        }
      }

      // Compute acceleration: compliance torque + damping
      v_ = v.tail(m_robot.na());
      a_des_ = Kp_.cwiseProduct(tau_ext_)
                - Kd_.cwiseProduct(v_);

      for(unsigned int i=0; i<active_axis_.size(); i++)
        constraint_.vector()(i) = a_des_(active_axis_(i));
      return constraint_;
    }

    bool TaskSkinJointCompliance::computeContactJacobian(const cc::SkinPatch& contact, Data & data, Matrix6x& Jc)
    {
      // get the kinematic frame
      if(!m_robot.model().existFrame(contact.frame()))
        return false;
      auto id = m_robot.model().getFrameId(contact.frame());

      // frame placement wrt joint
      const Frame& frame = m_robot.model().frames[id];
      const JointIndex joint_id = frame.parent;
      T_f_w_ = data.oMi[joint_id] * frame.placement;

      // contact placement of contact wrt joint
      T_c_f_.translation(contact.pose().linear());
      T_c_f_.rotation(contact.pose().angular().toRotationMatrix());

      // contact wrt world
      T_c_w_ = T_f_w_ * T_c_f_; 

      // build the local jacobian to the contact
      Jc.setZero();
      pinocchio::details::translateJointJacobian(
        m_robot.model(), data, joint_id, pinocchio::LOCAL, T_c_w_, data.J, Jc);
      return true;
    }

    void TaskSkinJointCompliance::callback(const control_core_msgs::SkinPatchesConstPtr& msg)
    {
      contacts_.resize(msg->patches.size());
      for(size_t i = 0; i < contacts_.size(); ++i)
      {
        contacts_[i] = msg->patches[i];
      }
      has_patches_callback_ = true;
    }

    void TaskSkinJointCompliance::reconfigureRequest(Config &config, uint32_t level)
    {
      params_.updateConfig(config, level);
      Kp_ = params_.kp_factor * params_.kp;
      Kd_ = params_.kd_factor * params_.kd;
      alpha_proximity_ = params_.alpha_proximity;
      W_des_.force().z() = params_.fd_normal;
    }

    std::shared_ptr<TaskBase> TaskSkinJointCompliance::Load(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& name,
      bool activate,
      bool verbose)
    {
      cc::Parameters parameter(nh, name);
      parameter.addRequired<std::string>("topic");
      parameter.addRequired<cc::VectorX>("kp");
      parameter.addRequired<cc::VectorX>("kd");
      parameter.addRequired<cc::Scalar>("weight");
      parameter.addRequired<cc::Scalar>("alpha_proximity");
      parameter.addRequired<cc::VectorX>("mask");
      if (!parameter.load())
      {
        ROS_ERROR("load_compliance_task: Failed to load parameters of '%s'", name.c_str());
        return nullptr;
      }

      unsigned int priority = 1;
      auto task = std::make_shared<tsid::tasks::TaskSkinJointCompliance>(
        name, tsid_wrapper.robotInterface().wrapper(), parameter.get<std::string>("topic"));
      
      cc::Scalar weight = parameter.get<cc::Scalar>("weight");
      cc::VectorX kp = parameter.get<cc::VectorX>("kp");
      cc::VectorX kd = parameter.get<cc::VectorX>("kd");
      task->Kp(kp);
      task->Kd(kd);
      task->setMask(parameter.get<cc::VectorX>("mask"));
      task->setAlphaProximity(parameter.get<cc::Scalar>("alpha_proximity"));
      task->subscribe(nh);

      if(verbose)
        ROS_INFO("Adding Task name='%s' :\t priority=%u weight=%.1e kp=%.2f, kd=%.2f", name.c_str(), priority, weight, kp[0], kd[0]);
      tsid_wrapper.loadTask(name, task, weight, priority, activate);
      return task;
    }
  }
}