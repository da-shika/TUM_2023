#include <tsid/robots/robot-wrapper.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ics_tsid_tasks/skin/task_skin_contact_soft.h>

#include <ics_tsid_common/utilities/conversions.h>

#include <control_core/ros/parameters.h>

namespace tsid
{
  namespace tasks
  {
    using namespace std;
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskSkinContactSoft::TaskSkinContactSoft(const std::string & name,
                                                 RobotWrapper & robot,
                                                 const double dt,
                                                 const std::string & contact_topic,
                                                 const std::string & patch_frame,
                                                 const std::string & motion_frame,
                                                 math::ConstRefVector mask):
      TaskMotion(name, robot),
      has_contact_(false),
      dt_(dt),
      contact_topic_(contact_topic),
      patch_frame_(patch_frame),
      motion_frame_(motion_frame),
      constraint_(name, 6, robot.nv()),
      motion_ref_(12,6)
    {
      // patch frame
      assert(robot.model().existFrame(patch_frame));
      Index patch_id = robot.model().getFrameId(patch_frame);
      Frame patch_model_frame = robot.model().frames[patch_id];

      // motion frame
      if(!motion_frame.empty())
      {
        ref_frame_ = REF_IN_MOTION_FRAME;

        assert(robot.model().existFrame(motion_frame));
        frame_id_ = robot.model().getFrameId(motion_frame);
        frame_ = robot.model().frames[frame_id_];

        // check motion frame and patch are mapped to the same joint (use same jacobian)
        assert(frame_.parent == patch_model_frame.parent);
      }
      else
      {
        ref_frame_ = REF_IN_CONTACT_FRAME;
        
        frame_id_ = patch_id;
        frame_ = robot.model().frames[frame_id_];
      }
      joint_id_ = frame_.parent;

      // store their relative transformation (patch frame w.r.t. motion frame)
      X_p_m_ = ics::to_pose(frame_.placement.actInv(patch_model_frame.placement));
      X_c_m_.setIdentity();
      R_m_w_.setIdentity();

      Kf_.setZero(6);
      Kp_.setZero(6);
      Kd_.setZero(6);
      Ki_.setZero(6);

      forceIntegralError_.setZero(6);
      p_error_vec_.setZero(6);
      v_error_vec_.setZero(6);
      f_error_vec_.setZero(6);
      
      force_ref_.setZero();
      v_ref_.setZero();
      a_ref_.setZero();
      T_ref_.setIdentity();
      T_m_w_.setIdentity();

      fext_.setZero();
      a_des_.setZero(6);
      v_vec_.setZero(6);
      J_.setZero(6, robot.nv());

      // default: fx, taux, tauy
      setMask(mask);
      proximity_weight_ = 0.0;
      leak_rate_ = 0.05;

      // skin
      contact_.setZero();
      skin_prox_.setZero();
      skin_wrench_.setZero();
      contact_.frame() = patch_frame;
    }

    bool TaskSkinContactSoft::connect(ros::NodeHandle& nh)
    {
      //////////////////////////////////////////////////////////////////////////
      // setup connection to wbc generator
      //////////////////////////////////////////////////////////////////////////
      bool ret = true;
      if(!cc::is_topic_published(contact_topic_, "control_core_msgs::SkinPatches"))
      {
        ROS_ERROR("TaskSkinContactSoft::connect: Topic '%s' not yet published", contact_topic_.c_str());
        ret = false;
      }
      wbc_contacts_sub_ = nh.subscribe(contact_topic_, 1, &TaskSkinContactSoft::callback, this);
      return ret;
    }

    void TaskSkinContactSoft::setForceReference(const Vector6 & ref) 
    {
      force_ref_ = ref;
    }

    void TaskSkinContactSoft::setMotionReference(const TrajectorySample & ref) {
      motion_ref_ = ref;
TSID_DISABLE_WARNING_PUSH
TSID_DISABLE_WARNING_DEPRECATED
      assert(ref.pos.size() == 12);
      T_ref_.translation( ref.pos.head<3>());
      T_ref_.rotation(MapMatrix3(&ref.pos(3), 3, 3));
TSID_DISABLE_WARNING_POP
      v_ref_ = Motion(ref.getDerivative());
      a_ref_ = Motion(ref.getSecondDerivative());
    }

    void TaskSkinContactSoft::setMask(math::ConstRefVector mask)
    {
      m_mask = mask;
      int n = dim();
      constraint_.resize(n, (unsigned int)J_.cols());
      drift_masked_.resize(n);
      a_des_masked_.resize(n);
    }

    void TaskSkinContactSoft::Kf(ConstRefVector Kf)
    {
      assert(Kf.size()==6); Kf_ = Kf;
    }

    void TaskSkinContactSoft::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==6); Kp_ = Kp;
    }

    void TaskSkinContactSoft::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==6); Kd_ = Kd;
    }

    void TaskSkinContactSoft::Ki(ConstRefVector Ki)
    {
      assert(Ki.size()==6); Ki_ = Ki;
    }

    const TaskSkinContactSoft::Vector & TaskSkinContactSoft::velocity() const
    {
      return v_vec_;
    }

    const TaskSkinContactSoft::Vector & TaskSkinContactSoft::getDesiredAcceleration() const
    {
      return a_des_masked_;
    }

    TaskSkinContactSoft::Vector TaskSkinContactSoft::getAcceleration(ConstRefVector dv) const
    {
      return constraint_.matrix()*dv + drift_masked_;
    }

    TaskSkinContactSoft::Index TaskSkinContactSoft::frame_id() const
    {
      return frame_id_;
    }

    const ConstraintBase & TaskSkinContactSoft::getConstraint() const
    {
      return constraint_;
    }

    const ConstraintBase & TaskSkinContactSoft::compute(const double ,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
    {
      a_des_.setZero();

      if(ref_frame_ == REF_IN_MOTION_FRAME)
      {
        // all quantities expressed in MOTION FRAME
        m_robot.framePosition(data, frame_id_, T_m_w_);
        m_robot.frameVelocity(data, frame_id_, v_);
        m_robot.frameClassicAcceleration(data, frame_id_, drift_);
        m_robot.frameJacobianLocal(data, frame_id_, J_);
        R_m_w_.rotation(T_m_w_.rotation());

        // motion
        errorInSE3(T_m_w_, T_ref_, p_error_);                                     
        p_error_vec_ = p_error_.toVector();
        v_error_ =  R_m_w_.actInv(v_ref_) - v_; 

        // force transformed in motion frame
        X_c_m_ = X_p_m_*contact_.pose();
        Ad_c_m_ = cc::adjointMatrixWrench(X_c_m_);
        skin_prox_ = Ad_c_m_*contact_.p().W();    // measured wrt c
        skin_wrench_ = Ad_c_m_*contact_.f().W();  // measured wrt c
        fext_ = proximity_weight_*skin_prox_ + (1 - proximity_weight_)*skin_wrench_;

        // compute the force component                                            
        f_error_vec_ = (force_ref_ - fext_);                                       
      }
      else
      {
        // all quantites expressed in CONTACT FRAME
        T_c_f_.translation() = contact_.pose().linear();
        T_c_f_.rotation() = contact_.pose().angular().toRotationMatrix();
        T_c_j_ = frame_.placement * T_c_f_;

        T_m_w_ = data.oMf[joint_id_] * T_c_j_;
        R_m_w_.rotation(T_m_w_.rotation());

        J_.setZero();
        pinocchio::details::translateJointJacobian(
          m_robot.model(), data, joint_id_, pinocchio::LOCAL, T_m_w_, data.J, J_);

        // velocity and bias acceleration in contact frame
        drift_ = T_c_j_.actInv(data.a[joint_id_]);
        v_ = T_c_j_.actInv(data.v[joint_id_]);
        drift_.linear() += v_.angular().cross(v_.linear());

        // motion in contact frame
        errorInSE3(T_m_w_, T_ref_, p_error_);                                     
        p_error_vec_ = p_error_.toVector();
        v_error_ =  R_m_w_.actInv(v_ref_) - v_; 

        // force
        fext_ = proximity_weight_*contact_.p().W() + (1 - proximity_weight_)*contact_.f().W();
      }

      // motion component
      a_des_ += (Kp_.cwiseProduct(p_error_vec_) 
                + Kd_.cwiseProduct(v_error_.toVector())
                + R_m_w_.actInv(a_ref_).toVector());

      // force component                                            
      f_error_vec_ = (force_ref_ - fext_);                                       

      a_des_ += (Kf_.cwiseProduct(f_error_vec_) 
                + Ki_.cwiseProduct(forceIntegralError_));
      forceIntegralError_ += (f_error_vec_ - leak_rate_ * forceIntegralError_) * dt_;

      //////////////////////////////////////////////////////////////////////////
      // mask unused dofs
      //////////////////////////////////////////////////////////////////////////
      int idx = 0;
      for (int i = 0; i < 6; i++) {
        if (m_mask(i) != 1.) continue;
        constraint_.matrix().row(idx) = J_.row(i);
        constraint_.vector().row(idx) = (a_des_ - drift_.toVector()).row(i);
        a_des_masked_(idx)            = a_des_(i);
        drift_masked_(idx)            = drift_.toVector()(i);
        idx += 1;
      }

      return constraint_;
    }

    void TaskSkinContactSoft::callback(const control_core_msgs::SkinPatchesConstPtr& msg)
    {
      // check if contact on given frame
      for(const auto& patch : msg->patches)
      {
        if(patch.header.frame_id == patch_frame_)
        {
          contact_ = patch;
          has_contact_ = true;
        }
      }

      // no contact found on patch_frame, set force feedback to zero
      if(!has_contact_)
      {                                               
        contact_.force().wrench().setZero();
        contact_.proximity().wrench().setZero();
      }
    }

    std::shared_ptr<TaskSkinContactSoft> TaskSkinContactSoft::Make(
      ics::TSIDWrapperInterface& tsid_wrapper,
      const std::string& name,
      cc::Scalar dt,
      const std::string & contact_topic,
      const std::string & patch_frame,
      const std::string & motion_frame,
      cc::Scalar weight,
      ConstRefVector kf,
      ConstRefVector kp,
      ConstRefVector kd,
      ConstRefVector ki,
      ConstRefVector mask,
      ros::NodeHandle& nh,
      bool activate,
      bool verbose)
    {
      auto& robot = tsid_wrapper.robotInterface().wrapper();

      unsigned int priority = (weight <= 0) ? 0 : 1;
      auto task = std::make_shared<tsid::tasks::TaskSkinContactSoft>(
        name, robot, dt, contact_topic, patch_frame, motion_frame, mask);
      
      task->Kf(kf);
      task->Kp(kp);
      task->Kd(kd);
      task->Ki(ki);
      task->setForceReference(cc::Vector6::Zero());

      std::string frame = motion_frame.empty() ? patch_frame : motion_frame;    // TODO: this is not correct!

      pinocchio::SE3 ref = robot.framePosition(
        tsid_wrapper.formulation().data(), robot.model().getFrameId(frame));
      auto sample = ics::to_sample(ref);
      task->setMotionReference(sample);
      task->connect(nh);

      tsid_wrapper.loadTask(name, task, weight, priority, activate);
      ROS_WARN_STREAM("TaskSkinContactSoft::Make: loadTask done");
      return task;
    }

    std::shared_ptr<TaskBase> TaskSkinContactSoft::Load(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& name,
      bool activate,
      bool verbose)
    {
      cc::Parameters parameter(nh, name);
      parameter.addRequired<cc::Scalar>("dt");
      parameter.addRequired<std::string>("contact_topic");
      parameter.addRequired<std::string>("patch_frame");
      parameter.addRequired<std::string>("motion_frame");
      parameter.addRequired<cc::Scalar>("weight");
      parameter.addRequired<cc::Vector6>("kf");
      parameter.addRequired<cc::Vector6>("kp");
      parameter.addRequired<cc::Vector6>("kd");
      parameter.addOptional<cc::Vector6>("ki", cc::Vector6::Zero());
      parameter.addOptional<cc::Vector6>("mask", cc::Vector6::Ones());
      if (!parameter.load())
      {
        ROS_ERROR("load_skin_force_equality_task: Failed to load parameters of '%s'", name.c_str());
        return nullptr;
      }
      cc::Scalar weight = parameter.get<cc::Scalar>("weight");
      unsigned int priority = (weight <= 0) ? 0 : 1;

      auto task = TaskSkinContactSoft::Make(
        tsid_wrapper, name, 
        parameter.get<cc::Scalar>("dt"), 
        parameter.get<std::string>("contact_topic"), 
        parameter.get<std::string>("patch_frame"), 
        parameter.get<std::string>("motion_frame"), 
        parameter.get<cc::Scalar>("weight"),
        parameter.get<cc::Vector6>("kf"), parameter.get<cc::Vector6>("kp"), 
        parameter.get<cc::Vector6>("kd"), parameter.get<cc::Vector6>("ki"), 
        parameter.get<cc::Vector6>("mask"), nh, activate, verbose);
      if(verbose)
        ROS_INFO("Adding Task name='%s' :\t priority=%u", name.c_str(), priority);
      return task;
    }

  }
}
