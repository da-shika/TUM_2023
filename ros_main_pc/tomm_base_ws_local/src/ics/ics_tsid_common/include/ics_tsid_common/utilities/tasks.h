#ifndef ICS_TSID_UTILITES_TASKS_H_
#define ICS_TSID_UTILITES_TASKS_H_

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/tasks/task-base.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/contacts/contact_6d_ext.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types.h>

////////////////////////////////////////////////////////////////////////////////
// std includes
////////////////////////////////////////////////////////////////////////////////
#include <unordered_map>

namespace ics
{
  typedef std::unordered_map<std::string, cc::Scalar> WeightMap;
  typedef std::unordered_map<std::string, std::shared_ptr<tsid::tasks::TaskBase> > TaskMap;
  typedef std::unordered_map<std::string, std::shared_ptr<tsid::contacts::Contact6dExt> > ContactMap;

  typedef std::vector<std::shared_ptr<tsid::tasks::TaskBase> > TaskVector;
  typedef std::vector<std::shared_ptr<tsid::contacts::Contact6dExt> > ContactVector;

  /**
   * @brief Get the Task object from Map, cast to correct type
   * 
   * @tparam T 
   * @param tasks 
   * @param name 
   * @return std::shared_ptr<T> 
   */
  template<typename T>
  inline std::shared_ptr<T> get_task(TaskMap& tasks, const std::string& name, bool verbose = true)
  {
    auto it = tasks.find(name);
    if(it == tasks.end())
    {
      if(verbose)
        ROS_ERROR("tasks::get_task: '%s' not found.", name.c_str());
      return nullptr;
    }
    std::shared_ptr<T> ptr = std::dynamic_pointer_cast<T>(it->second);
    if(!ptr)
    {
      if(verbose)
        ROS_ERROR("tasks::get_task: '%s' has incorrect type", name.c_str());
      return nullptr;
    }
    return ptr;
  }

  /**
   * @brief Get the Contact object from Map
   * 
   * @param contacts 
   * @param name 
   * @return std::shared_ptr<tsid::contacts::Contact6d> 
   */
  inline std::shared_ptr<tsid::contacts::Contact6dExt> get_contact(ContactMap& contacts, const std::string& name, bool verbose = true)
  {
    auto it = contacts.find(name);
    if(it == contacts.end())
    {
      if(verbose)
        ROS_ERROR("tasks::get_contact: '%s' not found.", name.c_str());
      return nullptr;
    }
    return it->second;
  }

  /**
   * @brief extracts the weight of a task from the tsid formulation
   * 
   * @param tsid 
   * @param name 
   * @return cc::Scalar 
   */
  inline cc::Scalar extract_task_weight(tsid::InverseDynamicsFormulationAccForce& formulation, const std::string& name)
  {
    tsid::solvers::ConstraintLevel::iterator it;
    for(unsigned int i = 1; i < formulation.m_hqpData.size(); ++i)
    {
      for(it=formulation.m_hqpData[i].begin(); it!=formulation.m_hqpData[i].end(); it++)
      {
        if(it->second->name() == name)
        {
          return it->first;
        }
      }
    }
    return -1;
  }

}

#endif