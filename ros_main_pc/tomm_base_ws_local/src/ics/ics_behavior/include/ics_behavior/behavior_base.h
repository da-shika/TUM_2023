#ifndef ICS_BEHAVIOR_BEHAVIOR_BASE_H_
#define ICS_BEHAVIOR_BEHAVIOR_BASE_H_

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/interfaces/generic_module_base.h>

////////////////////////////////////////////////////////////////////////////////
// ics_behavior includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_formulation/controller_base.h>

namespace ics
{

  /**
   * @brief BehaviorBase Class
   * 
   * BaseClass for all behaviors.
   * 
   * Behaviors are a collection of tasks that are loaded into the controller
   * by the behavior manager.
   * 
   * Behaviors require a "tasks" parameter to be set inside their namespace.
   * They are loaded within loadRequest() and removed within unloadRequest().
   * 
   * Funtionallity is implemented in start(), update(), stop(), publish()
   */
  class BehaviorBase : public cc::GenericModuleBase
  {
    public:
      typedef cc::GenericModuleBase Base;
      typedef ControllerBase Controller;
      typedef std::shared_ptr<Controller> ControllerPtr;
      typedef std::vector<std::string> Tasks;
    
    protected:
      bool is_loaded_;                    //!< behavior loaded tasks
      bool has_updated_once_;             //!< behavior update called once

      ros::NodeHandle nh_;            
      cc::Parameters params_;

      ControllerPtr controller_;          //!< controller pointer

    public:
      BehaviorBase(const std::string& name);
      virtual ~BehaviorBase();

      /**
       * @brief behavior tasks are loaded into controller
       */
      bool isLoaded() const { return is_loaded_; }

      /**
       * @brief Set the Controller object
       * 
       * Stores a pointer to the controller this task is using to command the 
       * robot.
       * 
       * @param controller 
       */
      virtual void setController(ControllerPtr controller) { 
        controller_ = controller; }

      //////////////////////////////////////////////////////////////////////////
      // interface functions
      //////////////////////////////////////////////////////////////////////////

      /** 
      * @brief initialize the Module after construction. 
      * 
      * Parameter holds the parameters requried for initialization.
      *
      * @param ros nh for private namespace
      * @param params parameters of the module
      */
      virtual bool initRequest(ros::NodeHandle& nh, Base::Parameters& params) override;

      /**
       * @brief Load the behavior into the controller.
       * 
       * Adds the tasks used by this behavior to the controller
       * Note: This needs to be called before start!
       */
      virtual bool loadRequest();

      /**
       * @brief start the behavior
       */
      virtual bool startRequest(const ros::Time &time);

      /**
       * @brief update the behavior
       */
      virtual bool updateRequest(const ros::Time &time, const ros::Duration &period);

      /**
       * @brief stop the state
       */
      virtual bool stopRequest(const ros::Time &time);

      /**
       * @brief remove the task from the controller
       */
      virtual bool unloadRequest();

      /**
       * @brief publish information
       */
      virtual void publishRequest(const ros::Time &time);

    protected:

      //////////////////////////////////////////////////////////////////////////
      // overloadable functions
      //////////////////////////////////////////////////////////////////////////

      virtual bool start(const ros::Time &time) { return true; }

      virtual bool load() { return true; }

      virtual bool update(const ros::Time &time, const ros::Duration &period) = 0;

      virtual bool stop(const ros::Time &time) { return true; }

      virtual bool unload() { return true; }

      virtual void publish(const ros::Time &time) {}


    private:  
      /**
       * @brief load as vector of tasks from parameter server
       * 
       */
      bool loadTasksFromParameters(ros::NodeHandle& nh, const std::vector<std::string> &names, bool activate, bool verbose);

      /**
       * @brief unload vector of tasks
       * 
       */
      bool unLoadTasks(const std::vector<std::string> &names, bool verbose);
  };

}


#endif