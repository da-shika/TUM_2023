#ifndef ICS_BEHAVIOR_BEHAVIOR_MANAGER_H_
#define ICS_BEHAVIOR_BEHAVIOR_MANAGER_H_

////////////////////////////////////////////////////////////////////////////////
// ics_behavior includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_behavior/behavior_base.h>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/interfaces/controller_base.h>
#include <control_core/utilities/thread.h>

#include <unordered_map>
#include <behavior_msgs/ChangeBehavior.h>
#include <behavior_msgs/ListBehavior.h>
#include <behavior_msgs/Timing.h>

namespace ics
{
  /**
   * @brief BehaviorManager Class
   * 
   * Holds a container of behaviors and acts like a Controller from the outside
   * 
   * Loads+Starts and Unloads+Stops behaviors via service calls.
   * 
   * Updates the controller after all active behaviors have set their references.
   * 
   * Note: updateRequest() function updates all active controllers and must be
   * triggert from the outside (currently in the control_plugin).
   * Publishing information to ros happens in a seperate thread, 
   * decoupled form the control computations.
   * 
   */
  class BehaviorManager : public cc::ControllerBase, public cc::Thread
  {
  public:
    typedef cc::ControllerBase Base;
    typedef cc::Thread Thread;

    typedef ics::ControllerBase Controller;
    typedef std::shared_ptr<Controller> ControllerPtr;

    typedef cc::ModuleBase Visualizer;
    typedef std::shared_ptr<Visualizer> VisualizerPtr;

    typedef std::shared_ptr<BehaviorBase> BehaviorPtr;
    typedef std::unordered_map<std::string, BehaviorPtr> BehaviorMap;

  private:
    bool verbose_;
    bool has_change_behavior_;                  // beahvior switch requried
    cc::Scalar publish_rate_;                   // publishing rate

    ////////////////////////////////////////////////////////////////////////////
    // controller
    ////////////////////////////////////////////////////////////////////////////
    ControllerPtr controller_;                  // pointer to the controller

    ////////////////////////////////////////////////////////////////////////////
    // visualizer / publishers
    ////////////////////////////////////////////////////////////////////////////
    VisualizerPtr visualizer_;                  // pointer to the visualizer 
    ros::Publisher timing_pub_;                 // controller timings
    behavior_msgs::Timing timing_msg_;          // controller timings msg
    std::int64_t loop_dur_;                     // control loop duration 

    ////////////////////////////////////////////////////////////////////////////
    // behaviors maps
    ////////////////////////////////////////////////////////////////////////////
    BehaviorMap behaviors_;                     // all registered behaviors
    BehaviorMap active_;                        // all active behaviors 
    
    ros::Publisher controller_stopped_pub_;     // controller stopped
    ros::ServiceServer change_behaviors_srv_;   // change behaviors
    ros::ServiceServer list_behaviors_srv_;     // list the loaded behaviors

    std::vector<std::string> start_behaviors_;  
    std::vector<std::string> stop_behaviors_;

  public:
    /**
     * @brief Construct a new Behavior Manager object
     * 
     * Requries a controller and optionally a visualizer
     */
    BehaviorManager(
      ControllerPtr controller, const std::string& name, 
      VisualizerPtr visualizer = nullptr, bool verbose=false);

    /**
     * @brief Destroy the Behavior Manager object
     * 
     */
    virtual ~BehaviorManager();

    /**
     * @brief add new behavior
     * 
     * Default: new behavior are inactive.
     * 
     * @param state the state
     * @param active sets as active or inactive
     * @return true 
     * @return false 
     */
    bool add(BehaviorPtr behavior, bool active = false);

    virtual const cc::JointState& command() const override {
      return controller_->command();}

  protected:
    ////////////////////////////////////////////////////////////////////////////
    // inferface
    ////////////////////////////////////////////////////////////////////////////

    virtual bool init(ros::NodeHandle& nh, Parameters& global_params) override;

    virtual void start(const ros::Time &time) override;

    virtual bool update(const ros::Time &time, const ros::Duration &period) override;

    virtual void stop(const ros::Time &time) override;

    virtual void publish(const ros::Time &time) override;

    ////////////////////////////////////////////////////////////////////////////
    // internal publisher thread
    ////////////////////////////////////////////////////////////////////////////
    private:
      virtual void run() override;

    ////////////////////////////////////////////////////////////////////////////
    // ros connections
    ////////////////////////////////////////////////////////////////////////////
    private:
      bool change(
        const ros::Time &time, 
        const std::vector<std::string>& start,
        const std::vector<std::string>& stop);

      BehaviorPtr has(const std::string& name);

    private:
      bool changeBehaviorHandler(
        behavior_msgs::ChangeBehaviorRequest& req,
        behavior_msgs::ChangeBehaviorResponse& res);

      bool listBehaviorHandler(
        behavior_msgs::ListBehaviorRequest& req,
        behavior_msgs::ListBehaviorResponse& res);
  };

}

#endif