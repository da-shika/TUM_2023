#include <ics_tsid_tasks/tasks/task_factory.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/ros/parameters.h>

namespace ics
{
  std::unordered_map<std::string, TaskFactory::CreateTask> TaskFactory::task_methods_;
  std::unordered_map<std::string, TaskFactory::CreateContact> TaskFactory::contact_methods_;

  bool TaskFactory::Register(const std::string& type, TaskFactory::CreateTask create_fnc)
  {
    auto it = task_methods_.find(type);
    if(it == task_methods_.end())
    {
      task_methods_[type] = create_fnc;
      return true;
    }
    return false;
  }

  bool TaskFactory::Register(const std::string& type, TaskFactory::CreateContact create_fnc)
  {
    auto it = contact_methods_.find(type);
    if(it == contact_methods_.end())
    {
      contact_methods_[type] = create_fnc;
      return true;
    }
    return false;
  }

  bool TaskFactory::hasTask(const std::string& type)
  {
    return cc::has(task_methods_, type);
  }

  bool TaskFactory::hasContact(const std::string& type)
  {
    return cc::has(contact_methods_, type);
  }

  void TaskFactory::List()
  {
    ROS_INFO("TaskFactory Loaded Tasks n=%ld", task_methods_.size());
    for(auto task : task_methods_)
      ROS_INFO("- '%s'", task.first.c_str());
    ROS_INFO("TaskFactory Loaded Contacts n=%ld", contact_methods_.size());
    for(auto contact : contact_methods_)
      ROS_INFO("- '%s'", contact.first.c_str());
  }

  std::shared_ptr<tsid::tasks::TaskBase> TaskFactory::LoadTask(
    TSIDWrapper& tsid_wrapper,
    ros::NodeHandle& nh,
    const std::string& type,
    const std::string& name,
    bool activate,
    bool verbose)
  {
    auto it = task_methods_.find(type);
    if(it != task_methods_.end())
    {
      auto task = it->second(tsid_wrapper, nh, name, activate, verbose);
      return task;
    }
    return nullptr;
  }

  std::shared_ptr<tsid::contacts::Contact6dExt> TaskFactory::LoadContact(
    TSIDWrapper& tsid_wrapper,
    ros::NodeHandle& nh,
    const std::string& type,
    const std::string& name,
    bool activate,
    bool verbose)
  {
    auto it = contact_methods_.find(type);
    if(it != contact_methods_.end())
    {
      auto contact = it->second(tsid_wrapper, nh, name, activate, verbose);
      return contact;
    }
    return nullptr;
  }

  bool TaskFactory::LoadList(
    TSIDWrapper& tsid_wrapper,
    ros::NodeHandle& nh,
    const std::vector<std::string>& names,
    bool activate,
    bool verbose)
  {
    // load
    for (const auto &name : names)
    {
      // load the task type and check if already in formulation
      std::string type;
      if (!cc::load(nh.getNamespace() + "/" + name + "/" + "type", type))
      {
        ROS_ERROR("load_task_list(): Can't find type for '%s', aborting", name.c_str());
        return false;
      }
      
      if(hasTask(type))
      {
        // load as task
        if(tsid_wrapper.hasTask(name) && activate)
        {
          if(!tsid_wrapper.addTask(name))
          {
            ROS_WARN("load_task_list(): Task '%s' already loaded, but adding.", name.c_str());
          }
          continue;
        }
        auto task = LoadTask(tsid_wrapper, nh, type, name, activate, verbose);
        if(!task)
          return false;
      }
      else if(hasContact(type))
      {
        // load as contact
        if(tsid_wrapper.hasContact(name))
        {
          ROS_WARN("load_task_list(): Contact '%s' already loaded, skipping.", name.c_str());
          continue;
        }
        auto contact = LoadContact(tsid_wrapper, nh, type, name, false, verbose);
        if(!contact)
          return false;
      }
      else
      {
        ROS_ERROR("TaskFactory::LoadList(): Unknown type '%s', aborting", type.c_str());
        return false;
      }
    }
    return true;
  }

}