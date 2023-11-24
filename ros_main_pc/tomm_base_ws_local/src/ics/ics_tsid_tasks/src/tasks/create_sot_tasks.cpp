////////////////////////////////////////////////////////////////////////////////
// ics_tsid_tasks includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_tasks/tasks/create_sot_tasks.h>

////////////////////////////////////////////////////////////////////////////////
// ics_tsid_tasks includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/utilities/conversions.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/ros/parameters.h>

namespace ics
{

  std::shared_ptr<tsid::tasks::TaskSE3Equality> make_se3_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name,
    const std::string& frame,
    cc::Scalar weight,
    cc::Scalar kp,
    cc::Scalar kd,
    const cc::Vector6& mask,
    bool activate,
    bool verbose)
  {
    auto &robot = tsid_wrapper.robotInterface().wrapper();
    if(!robot.model().existFrame(frame))
    {
      ROS_ERROR("make_se3_task %s: frame '%s' does not exist", name.c_str(), frame.c_str());
      return nullptr;
    }

    unsigned int priority = 1;
    auto task = std::make_shared<tsid::tasks::TaskSE3Equality>(name, robot, frame);

    cc::Vector6 kp_vec, kd_vec;
    kp_vec.head(3).setConstant(kp);
    kp_vec.tail(3).setConstant(0.5*kp);
    kd_vec = 2.0 * kp_vec.cwiseSqrt();

    task->Kp(kp_vec);
    task->Kd(kd_vec);
    task->setMask(mask);

    // ref
    pinocchio::SE3 ref = robot.framePosition(tsid_wrapper.data(), robot.model().getFrameId(frame));
    auto sample = ics::to_sample(ref);
    task->setReference(sample);

    if(verbose)
      ROS_INFO("Adding Task name='%s' :\t priority=%u weight=%.1e, kp=%.2f, kd=%.2f", name.c_str(), priority, weight, kp, kd);

    tsid_wrapper.loadTask(name, task, weight, priority, activate);
    return task;
  }

  std::shared_ptr<tsid::tasks::TaskComEquality> make_com_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name,
    cc::Scalar weight,
    cc::Scalar kp,
    cc::Scalar kd,
    const cc::Vector3& mask,
    bool activate,
    bool verbose)

  {
    auto &robot = tsid_wrapper.robotInterface().wrapper();
    unsigned int priority = 1;

    auto task = std::make_shared<tsid::tasks::TaskComEquality>(name, robot);
    if(kd<0)
      kd = 2.0*std::sqrt(kp);
    task->Kp(kp*cc::Vector3::Ones());
    task->Kd(kd*cc::Vector3::Ones());
    task->setMask(mask);

    // set reference
    task->setReference(ics::to_sample(robot.com(tsid_wrapper.data())));

    if(verbose)
      ROS_INFO("Adding Task name='%s' :\t priority=%u weight=%.1e, kp=%.2f, kd=%.2f", name.c_str(), priority, weight, kp, kd);
    tsid_wrapper.loadTask(name, task, weight, priority, activate);
    return task;
  }

  std::shared_ptr<tsid::tasks::TaskAMEquality> make_momentum_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name,
    cc::Scalar weight,
    cc::Scalar kp,
    cc::Scalar kd,
    const cc::Vector3& mask,
    bool activate,
    bool verbose)
  {
    auto &robot = tsid_wrapper.robotInterface().wrapper();
    unsigned int priority = 1;

    auto task = std::make_shared<tsid::tasks::TaskAMEquality>(name, robot);
    if(kd<0)
      kd = 2.0*std::sqrt(kp);
    task->Kp(kp*cc::Vector3::Ones());
    task->Kd(kd*cc::Vector3::Ones());
    task->setMask(mask);

    // set reference
    task->setReference(ics::to_sample(cc::Vector3::Zero()));

    if(verbose)
      ROS_INFO("Adding Task name='%s' :\t priority=%u weight=%.1e, kp=%.2f, kd=%.2f", name.c_str(), priority, weight, kp, kd);
    tsid_wrapper.loadTask(name, task, weight, priority, activate);
    return task;
  }

  std::shared_ptr<tsid::tasks::TaskJointPosture> make_posture_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name,
    const cc::JointPosition& q_posture,
    cc::Scalar weight,
    const cc::VectorX& mask,
    const cc::VectorX& kp,
    cc::Scalar kd,
    bool activate,
    bool verbose)
  {
    auto &robot = tsid_wrapper.robotInterface().wrapper();
    unsigned int priority = 1;

    auto task = std::make_shared<tsid::tasks::TaskJointPosture>(name, robot);
    task->Kp(kp);
    if(kd < 0)
      task->Kd(2.0*task->Kp().cwiseSqrt());
    else
      task->Kd(kd*cc::VectorX::Ones(robot.na()));
    task->setMask(mask);

    // set reference
    task->setReference(ics::to_sample(q_posture));

    if(verbose)
      ROS_INFO("Adding Task name='%s' :\t priority=%u weight=%.1e kp=%.2f, kd=%.2f", name.c_str(), priority, weight, kp[0], task->Kd()[0]);
    tsid_wrapper.loadTask(name, task, weight, priority, activate);
    return task;
  }

  std::shared_ptr<tsid::tasks::TaskJointPosVelAccBounds> make_bounds(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name,
    cc::Scalar dt,
    cc::Scalar weight,
    const cc::VectorX& mask,
    bool activate,
    bool verbose)
  {
    auto &robot = tsid_wrapper.robotInterface().wrapper();
    unsigned int priority = (weight <= 0) ? 0 : 1;

    auto task = std::make_shared<tsid::tasks::TaskJointPosVelAccBounds>(name, robot, dt, false);
    auto dq_max = robot.model().velocityLimit.tail(robot.na());
    auto ddq_max = dq_max / dt;
    task->setVelocityBounds(dq_max);
    task->setAccelerationBounds(ddq_max);
    auto q_lb = robot.model().lowerPositionLimit.tail(robot.na());
    auto q_ub = robot.model().upperPositionLimit.tail(robot.na());
    task->setPositionBounds(q_lb, q_ub);
    task->setMask(mask);

    if(verbose)
      ROS_INFO("Adding Bounds name='%s'\t priority=%u v_max=%f, a_max=%f", name.c_str(), priority, dq_max[0], ddq_max[0]);
    tsid_wrapper.loadTask(name, task, weight, priority, activate);
    return task;
  }

  std::shared_ptr<tsid::contacts::Contact6dExt> make_contact_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name, 
    const std::string& joint_name,
    cc::Scalar weight,
    cc::Scalar weight_reg,
    cc::Scalar lxn, cc::Scalar lyn, cc::Scalar lxp, cc::Scalar lyp, cc::Scalar lz,
    cc::Scalar mu, const cc::Vector3& contact_normal, cc::Scalar fmin, cc::Scalar fmax,
    cc::Scalar kp,
    cc::Scalar kd,
    bool activate,
    bool verbose)
  {
    auto &robot = tsid_wrapper.robotInterface().wrapper();

    /* check
    if(!robot.model().existJointName(joint_name))
    {
      ROS_ERROR("make_contact_task: joint '%s' does not exist", joint_name.c_str());
      return nullptr;
    }*/

    // setup points in anti-clockwise direction 
    cc::MatrixX contact_points(3,4);
    contact_points << 
      -lxn,  lxp, lxp, -lxn,
      -lyn, -lyn, lyp,  lyp,
       -lz,  -lz, -lz,  -lz;

    unsigned int priority = (weight <= 0) ? 0 : 1;
    auto contact_task = std::make_shared<tsid::contacts::Contact6dExt>(
      name, robot, joint_name, contact_points, contact_normal, mu, fmin, fmax, 
      weight_reg, weight, priority);
    if(kd<0)
      kd = 2.0*std::sqrt(kp);
    contact_task->Kp(kp*cc::Vector6::Ones());
    contact_task->Kd(kd*cc::Vector6::Ones());
    
    auto ref = robot.framePosition(tsid_wrapper.data(), robot.model().getFrameId(joint_name));
    contact_task->setMotionReference(ics::to_sample(ref));

    //auto contact_ref = robot.position(formulation.data(), robot.model().getJointId(joint_name));
    //contact_task->setReference(contact_ref);

    if(verbose)
    {
      ROS_INFO("Adding Contact name='%s' :\t priority=%u, weight=%.1e, kp=%.2f, kd=%.2f", 
        name.c_str(), priority, weight, kp, kd);
    }
    // add the contact
    if(tsid_wrapper.loadContact(name, contact_task, 0, activate)) 
      ROS_INFO("Activating Contact name='%s'", name.c_str());
    return contact_task;
  }

  //----------------------------------------------------------------------------

  std::shared_ptr<tsid::tasks::TaskBase> load_se3_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    ros::NodeHandle &nh,
    const std::string &name,
    bool activate,
    bool verbose)
  {
    cc::Parameters parameter(nh, name);
    parameter.addRequired<std::string>("frame");
    parameter.addRequired<cc::Scalar>("kp");
    parameter.addOptional<cc::Scalar>("kd", -1);
    parameter.addRequired<cc::Scalar>("weight");
    parameter.addRequired<cc::Vector6>("mask");
    if (!parameter.load())
    {
      ROS_ERROR("load_se3_task: Failed to load parameters of '%s'", name.c_str());
      return nullptr;
    }
    return make_se3_task(
      tsid_wrapper, name, parameter.get<std::string>("frame"), parameter.get<cc::Scalar>("weight"),
      parameter.get<cc::Scalar>("kp"), parameter.get<cc::Scalar>("kd"), 
      parameter.get<cc::Vector6>("mask"), activate, verbose);
  }

  std::shared_ptr<tsid::tasks::TaskBase> load_com_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    ros::NodeHandle &nh,
    const std::string &name,
    bool activate,
    bool verbose)
  {
    cc::Parameters parameter(nh, name);
    parameter.addRequired<cc::Scalar>("kp");
    parameter.addOptional<cc::Scalar>("kd", 1);
    parameter.addRequired<cc::Scalar>("weight");
    parameter.addRequired<cc::Vector3>("mask");
    if (!parameter.load())
    {
      ROS_ERROR("load_com_task: Failed to load parameters of '%s'", name.c_str());
      return nullptr;
    }
    return make_com_task(tsid_wrapper, name, parameter.get<cc::Scalar>("weight"), 
      parameter.get<cc::Scalar>("kp"), parameter.get<cc::Scalar>("kd"), 
      parameter.get<cc::Vector3>("mask"), activate, verbose);
  }

  std::shared_ptr<tsid::tasks::TaskBase> load_momentum_task(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle &nh,
      const std::string &name,
      bool activate,
      bool verbose)
  {
    cc::Parameters parameter(nh, name);
    parameter.addRequired<cc::Scalar>("kp");
    parameter.addOptional<cc::Scalar>("kd", -1);
    parameter.addRequired<cc::Scalar>("weight");
    parameter.addRequired<cc::Vector3>("mask");
    if (!parameter.load())
    {
      ROS_ERROR("load_momentum_task: Failed to load parameters of '%s'", name.c_str());
      return nullptr;
    }
    return make_momentum_task(tsid_wrapper, name, parameter.get<cc::Scalar>("weight"),
      parameter.get<cc::Scalar>("kp"), parameter.get<cc::Scalar>("kd"), 
      parameter.get<cc::Vector3>("mask"), activate, verbose);
  }

  std::shared_ptr<tsid::tasks::TaskBase> load_posture_task(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle &nh,
      const std::string &name,
      bool activate,
      bool verbose)
  {
    cc::Parameters parameter(nh, name);
    parameter.addRequired<cc::VectorX>("kp");
    parameter.addOptional<cc::Scalar>("kd", -1);
    parameter.addRequired<cc::Scalar>("weight");
    parameter.addRequired<cc::VectorX>("posture");
    parameter.addRequired<cc::VectorX>("mask");
    if (!parameter.load())
    {
      ROS_ERROR("load_posture_task: Failed to load parameters of '%s'", name.c_str());
      return nullptr;
    }
    return make_posture_task(tsid_wrapper, name, parameter.get<cc::VectorX>("posture"),
      parameter.get<cc::Scalar>("weight"), parameter.get<cc::VectorX>("mask"),
      parameter.get<cc::VectorX>("kp"), parameter.get<cc::Scalar>("kd"), activate, verbose);
  }

  std::shared_ptr<tsid::tasks::TaskBase> load_bounds(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle &nh,
      const std::string &name,
      bool activate,
      bool verbose)
  {
    cc::Parameters parameter(nh, name);
    parameter.addRequired<cc::Scalar>("dt");
    parameter.addRequired<cc::Scalar>("weight");
    parameter.addRequired<cc::VectorX>("mask");
    if (!parameter.load())
    {
      ROS_ERROR("load_bounds: Failed to load parameters of '%s'", name.c_str());
      return nullptr;
    }
    return make_bounds(tsid_wrapper, name, parameter.get<cc::Scalar>("dt"),
                       parameter.get<cc::Scalar>("weight"), parameter.get<cc::VectorX>("mask"), activate, verbose);
  }

  std::shared_ptr<tsid::contacts::Contact6dExt> load_contact_task(
      ics::TSIDWrapperInterface& tsid_wrapper,
      ros::NodeHandle &nh,
      const std::string &name,
      bool activate,
      bool verbose)
  {
    cc::Parameters parameter(nh, name);
    parameter.addRequired<std::string>("joint");
    parameter.addRequired<cc::Scalar>("kp");
    parameter.addOptional<cc::Scalar>("kd", -1);
    parameter.addOptional<cc::Scalar>("weight", -1);
    parameter.addRequired<cc::Scalar>("weight_reg");
    parameter.addRequired<cc::Scalar>("lxn");
    parameter.addRequired<cc::Scalar>("lyn");
    parameter.addRequired<cc::Scalar>("lxp");
    parameter.addRequired<cc::Scalar>("lyp");
    parameter.addRequired<cc::Scalar>("lz");
    parameter.addRequired<cc::Scalar>("mu");
    parameter.addRequired<cc::Vector3>("normal");
    parameter.addRequired<cc::Scalar>("fmin");
    parameter.addRequired<cc::Scalar>("fmax");
    parameter.addRequired<cc::Vector6>("mask");
    if (!parameter.load())
    {
      ROS_ERROR("load_bounds: Failed to load parameters of '%s'", name.c_str());
      return nullptr;
    }
    return make_contact_task(tsid_wrapper, name, 
      parameter.get<std::string>("joint"), parameter.get<cc::Scalar>("weight"),
      parameter.get<cc::Scalar>("weight_reg"), parameter.get<cc::Scalar>("lxn"),
      parameter.get<cc::Scalar>("lyn"), parameter.get<cc::Scalar>("lxp"),
      parameter.get<cc::Scalar>("lyp"), parameter.get<cc::Scalar>("lz"), 
      parameter.get<cc::Scalar>("mu"), parameter.get<cc::Vector3>("normal"),
      parameter.get<cc::Scalar>("fmin"), parameter.get<cc::Scalar>("fmax"), 
      parameter.get<cc::Scalar>("kp"), parameter.get<cc::Scalar>("kd"), activate, verbose);
  }

}