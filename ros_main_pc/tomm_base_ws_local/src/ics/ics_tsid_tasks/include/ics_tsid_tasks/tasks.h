#ifndef ICS_TSID_TASKS_TASKS_H_
#define ICS_TSID_TASKS_TASKS_H_

////////////////////////////////////////////////////////////////////////////////
// Using Pinocchio with Fast collision lib
////////////////////////////////////////////////////////////////////////////////
#define PINOCCHIO_WITH_HPP_FCL

////////////////////////////////////////////////////////////////////////////////
// ics_tsid_tasks includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_tasks/tasks/create_sot_tasks.h>

// modified tsid tasks
#include <ics_tsid_tasks/tsid/task_manipulability.h>
#include <ics_tsid_tasks/tsid/task_self_collision_potential.h>
#include <ics_tsid_tasks/tsid/task_self_collision_constraint.h>
#include <ics_tsid_tasks/tsid/task_joint_posture_mobile.h>
#include <ics_tsid_tasks/tsid/task_joint_pos_vel_acc_bounds_mobile.h>

// skin related tasks
#include <ics_tsid_tasks/skin/task_skin_joint_compliance.h>
#include <ics_tsid_tasks/skin/task_skin_contact_soft.h>

#include <ics_tsid_tasks/skin/task_skin_distance_constraint_soft.h>
#include <ics_tsid_tasks/skin/task_skin_distance_constraint_hard.h>

#include <ics_tsid_tasks/skin/task_contact_distance_constraint_soft.h>
#include <ics_tsid_tasks/skin/task_contact_distance_constraint_hard.h>

#include <ics_tsid_tasks/skin/task_skin_force_constraint.h>
#include <ics_tsid_tasks/skin/task_contact_force_constraint_hard.h>

// organization
#include <ics_tsid_tasks/tasks/task_factory.h>

namespace ics
{

  /**
   * @brief call this function somewhere in the beginning of the application
   * Registers all Load function to the factory.
   * 
   * @return true 
   * @return false 
   */
  inline bool register_task_types()
  {
    bool ok = true;

    ok &=   TaskFactory::Register("se3", load_se3_task);
    ok &=   TaskFactory::Register("com", load_com_task);
    ok &=   TaskFactory::Register("moment", load_momentum_task);
    ok &=   TaskFactory::Register("posture", load_posture_task);
    ok &=   TaskFactory::Register("bound", load_bounds);
    ok &=   TaskFactory::Register("contact", load_contact_task);

    // custom tsid tasks
    ok &=   TaskFactory::Register("manipulability", tsid::tasks::TaskManipulability::Load);
    ok &=   TaskFactory::Register("posture_mobile", tsid::tasks::TaskJointPostureMobile::Load);
    ok &=   TaskFactory::Register("bound_mobile", tsid::tasks::TaskJointPosVelAccBoundsMobile::Load);
    ok &=   TaskFactory::Register("self_collision_potential", tsid::tasks::TaskSelfCollisionPotential::Load);
    ok &=   TaskFactory::Register("self_collision_constraint", tsid::tasks::TaskSelfCollisionConstraint::Load);

    ok &=   TaskFactory::Register("skin_joint_compliance", tsid::tasks::TaskSkinJointCompliance::Load);
    ok &=   TaskFactory::Register("skin_contact_soft", tsid::tasks::TaskSkinContactSoft::Load);
    ok &=   TaskFactory::Register("skin_distance_constraint_hard", tsid::tasks::TaskSkinDistanceConstraintHard::Load);
    ok &=   TaskFactory::Register("skin_distance_constraint_soft", tsid::tasks::TaskSkinDistanceConstraintSoft::Load);
    ok &=   TaskFactory::Register("contact_distance_constraint_soft", tsid::tasks::TaskContactDistanceConstraintSoft::Load);
    ok &=   TaskFactory::Register("contact_distance_constraint_hard", tsid::tasks::TaskContactDistanceConstraintHard::Load);
    ok &=   TaskFactory::Register("skin_force_constraint", tsid::tasks::TaskSkinForceConstraint::Load);
    ok &=   TaskFactory::Register("contact_force_constraint_hard", tsid::tasks::TaskContactForceConstraintHard::Load);
    
    return ok;
  }

}

#endif