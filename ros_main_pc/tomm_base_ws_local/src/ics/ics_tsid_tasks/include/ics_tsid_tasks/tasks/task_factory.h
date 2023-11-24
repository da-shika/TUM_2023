#ifndef ICS_TSID_TASKS_TASK_FACTORY_H_
#define ICS_TSID_TASKS_TASK_FACTORY_H_

////////////////////////////////////////////////////////////////////////////////
// ics includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/interfaces/tsid_wrapper_interface.h>
#include <ics_tsid_common/utilities/tasks.h>
#include <ics_tsid_common/contacts/contact_6d_ext.h>

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/tasks/task-base.hpp>

////////////////////////////////////////////////////////////////////////////////
// ros includes
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <unordered_map>

namespace ics
{

  /**
   * @brief TaskFactory Class
   * 
   * Static Class that allowes loading of tasks and contacts via 
   * parameter server
   * 
   * New task types need to be added via the Register() function before
   * calling LoadTask() or LoadContact() to load a task/contact.
   * See: sot_tasks/tasks.h
   * 
   * A list of tasks and contacts can be loaded via the function LoadList()
   * 
   */
  class TaskFactory
  {
  public:
    typedef ics::TSIDWrapperInterface TSIDWrapper;
    typedef ics::TaskMap TaskMap;
    typedef ics::ContactMap ContactMap;
    typedef ics::WeightMap WeightMap;

  public:
    /**
     * @brief function pointer for Tasks
     */
    using CreateTask = std::shared_ptr<tsid::tasks::TaskBase>(*)(
      TSIDWrapper& tsid_wrapper,
      ros::NodeHandle&,
      const std::string&,
      bool,
      bool);

    /**
     * @brief function pointer for Contacts
     */
    using CreateContact = std::shared_ptr<tsid::contacts::Contact6dExt>(*)(
      TSIDWrapper& tsid_wrapper,
      ros::NodeHandle&,
      const std::string&,
      bool,
      bool);

  private:
    static std::unordered_map<std::string, CreateTask> task_methods_;
    static std::unordered_map<std::string, CreateContact> contact_methods_;

  public:
    TaskFactory() = delete;

    /**
     * @brief register a new task by name with its create function
     * 
     * @param type 
     * @param create_fnc 
     * @return true 
     * @return false 
     */
    static bool Register(const std::string& type, CreateTask create_fnc);

    /**
     * @brief register a new contact by name with its create function
     * 
     * @param type 
     * @param create_fnc 
     * @return true 
     * @return false 
     */
    static bool Register(const std::string& type, CreateContact create_fnc);

    /**
     * @brief check if task type known
     * 
     * @param type 
     * @return true 
     * @return false 
     */
    static bool hasTask(const std::string& type);

    /**
     * @brief check if contact type known
     * 
     * @param type 
     * @return true 
     * @return false 
     */
    static bool hasContact(const std::string& type);

    /**
     * @brief list all loaded tasks
     * 
     * @return std::string 
     */
    static void List();

    /**
     * @brief Load a task form namespace 'name' and add it to the controller
     * 
     * Note: tasks get activated per default 
     * 
     * @param controller 
     * @param robot 
     * @param nh 
     * @param type 
     * @param name 
     * @param verbose 
     * @return std::shared_ptr<tsid::tasks::TaskBase> 
     */
    static std::shared_ptr<tsid::tasks::TaskBase> LoadTask(
      TSIDWrapper& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& type,
      const std::string& name,
      bool activate = true,
      bool verbose = true);

    /**
     * @brief Load a contact form namespace 'name' and add it to the controller
     * 
     * Note: contacts are deactivated per default
     * 
     * @param controller 
     * @param robot 
     * @param nh 
     * @param type 
     * @param name 
     * @param activate 
     * @param verbose 
     * @return std::shared_ptr<tsid::contacts::Contact6dExt> 
     */
    static std::shared_ptr<tsid::contacts::Contact6dExt> LoadContact(
      TSIDWrapper& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::string& type,
      const std::string& name,
      bool activate = false,
      bool verbose = true);

    /**
     * @brief Load a list of tasks and contacts and add them to he controller
     * 
     * Note: Contacts are deactivated upon loading
     * 
     * @param controller 
     * @param robot 
     * @param tasks 
     * @param contacts 
     * @param weights 
     * @param nh 
     * @param names 
     * @param verbose 
     * @return true 
     * @return false 
     */
    static bool LoadList(
      TSIDWrapper& tsid_wrapper,
      ros::NodeHandle& nh,
      const std::vector<std::string>& names,
      bool activate = true,
      bool verbose = true);
  };

}

#endif