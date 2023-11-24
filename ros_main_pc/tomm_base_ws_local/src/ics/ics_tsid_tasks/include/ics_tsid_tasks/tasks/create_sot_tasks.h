#ifndef ICS_TSID_TASKS_CREATE_SOT_TASKS_
#define ICS_TSID_TASKS_CREATE_SOT_TASKS_

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-angular-momentum-equality.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-bounds.hpp>
#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-se3-equality.hpp>

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/interfaces/tsid_wrapper_interface.h>
#include <ics_tsid_common/contacts/contact_6d_ext.h>

////////////////////////////////////////////////////////////////////////////////
// ros includes
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>

/**
 * @brief functions to load the existing default sot tasks
 */
namespace ics
{
  //////////////////////////////////////////////////////////////////////////////
  // create task programatically
  //////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<tsid::tasks::TaskSE3Equality> make_se3_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name,
    const std::string& frame,
    cc::Scalar weight,
    cc::Scalar kp,
    cc::Scalar kd=-1,
    const cc::Vector6& mask=cc::Vector6::Ones(),
    bool activate = true,
    bool verbose = true);

  std::shared_ptr<tsid::tasks::TaskComEquality> make_com_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name,
    cc::Scalar weight,
    cc::Scalar kp,
    cc::Scalar kd=-1,
    const cc::Vector3& mask=cc::Vector3::Ones(),
    bool activate = true,
    bool verbose = true);

  std::shared_ptr<tsid::tasks::TaskAMEquality> make_momentum_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name,
    cc::Scalar weight,
    cc::Scalar kp,
    cc::Scalar kd=-1,
    const cc::Vector3& mask=cc::Vector3::Ones(),
    bool activate = true,
    bool verbose = true);

  std::shared_ptr<tsid::tasks::TaskJointPosture> make_posture_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name,
    const cc::JointPosition& q_posture,
    cc::Scalar weight,
    const cc::VectorX& mask,
    const cc::VectorX& kp,
    cc::Scalar kd=-1,
    bool activate = true,
    bool verbose = true);

  std::shared_ptr<tsid::tasks::TaskJointPosVelAccBounds> make_bounds(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name,
    cc::Scalar dt,
    cc::Scalar weight,
    const cc::VectorX& mask,
    bool activate = true,
    bool verbose = true);

  std::shared_ptr<tsid::contacts::Contact6dExt> make_contact_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    const std::string& name, 
    const std::string& joint_name,
    cc::Scalar weight,
    cc::Scalar weight_reg,
    cc::Scalar lxn, cc::Scalar lyn, cc::Scalar lxp, cc::Scalar lyp, cc::Scalar lz,
    cc::Scalar mu, const cc::Vector3& contact_normal, cc::Scalar fmin, cc::Scalar fmax,
    cc::Scalar kp,
    cc::Scalar kd=-1,
    bool activate = false,                                                      // Note: contacts are special, they shoudn't be active after loading
    bool verbose = true);

  //////////////////////////////////////////////////////////////////////////////
  // create task from ros
  //////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<tsid::tasks::TaskBase> load_se3_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    ros::NodeHandle& nh,
    const std::string& name,
    bool activate = true,
    bool verbose = true);

  std::shared_ptr<tsid::tasks::TaskBase> load_com_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    ros::NodeHandle& nh,
    const std::string& name,
    bool activate = true,
    bool verbose = true);

  std::shared_ptr<tsid::tasks::TaskBase> load_momentum_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    ros::NodeHandle& nh,
    const std::string& name,
    bool activate = true,
    bool verbose = true);

  std::shared_ptr<tsid::tasks::TaskBase> load_posture_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    ros::NodeHandle& nh,
    const std::string& name,
    bool activate = true,
    bool verbose = true);

  std::shared_ptr<tsid::tasks::TaskBase> load_bounds(
    ics::TSIDWrapperInterface& tsid_wrapper,
    ros::NodeHandle& nh,
    const std::string& name,
    bool activate = true,
    bool verbose = true);

  std::shared_ptr<tsid::contacts::Contact6dExt> load_contact_task(
    ics::TSIDWrapperInterface& tsid_wrapper,
    ros::NodeHandle& nh,
    const std::string& name,
    bool activate = false,
    bool verbose = true);
}

#endif