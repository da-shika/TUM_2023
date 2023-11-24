#ifndef ICS_BEHAVIOR_ACTION_BEHAVIOR_BASE_H_
#define ICS_BEHAVIOR_ACTION_BEHAVIOR_BASE_H_

////////////////////////////////////////////////////////////////////////////////
// ics_behavior includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_behavior/behavior_base.h>

////////////////////////////////////////////////////////////////////////////////
// actionlib includes
////////////////////////////////////////////////////////////////////////////////
#include <actionlib/server/simple_action_server.h>

namespace ics
{

  /**
   * @brief ActionBehaviorBase
   * 
   * Baseclass for behaviors that are conntrolled through ros actionlib
   * 
   * Important: After initialization the start() function of an action is only 
   * called once a new goal is recived.
   * 
   * The overloaded update() function is reponsible to determine the end of
   * an action by calling server_->setSucceeded().
   * 
   * Optionally: Overload onPreemptRequest() to allow/prepare action stopping.
   * 
   * @note There is only one Goal active at a time
   * 
   * @tparam Action actionlib Action Type
   */
  template<class Action>
  class ActionBehaviorBase : public BehaviorBase
  {
  public:
    typedef BehaviorBase Base;
    using SimpleActionServer = actionlib::SimpleActionServer<Action>;
    using Goal = typename Action::_action_goal_type::_goal_type;
    using Result = typename Action::_action_result_type::_result_type;
    using Feedback = typename Action::_action_feedback_type::_feedback_type;

  protected:
    bool verbose_;
    std::unique_ptr<SimpleActionServer> server_;
    Goal goal_;
    Result result_;
    Feedback feedback_;
  
  public:
    ActionBehaviorBase(const std::string& name, bool verbose=false) : 
      Base(name),
      verbose_(verbose)
    {
    }

    virtual ~ActionBehaviorBase()
    {
    }

    virtual bool initRequest(ros::NodeHandle& nh, cc::Parameters& params) override
    {
      server_ = std::make_unique<SimpleActionServer>(nh, Base::name(), false);
      server_->registerGoalCallback(boost::bind(&ActionBehaviorBase::goalCallback, this));
      return Base::initRequest(nh, params);
    }

    virtual bool startRequest(const ros::Time &time) override
    {
      // at this point we can only start the server, derived start() not called
      server_->start();
      return true;
    }

    virtual bool updateRequest(const ros::Time &time, const ros::Duration &period) override
    {
      if(!Base::isRunning() && server_->isActive())
      {
        // active goal, turn on behavior
        if(!Base::startRequest(time))
        {
          PRINT_ERROR("'%s' failed to start action", Base::name().c_str());
          server_->setAborted();
        }
      }
      if(Base::isRunning() && !server_->isActive())
      {
        // goal reached, turn off behavior
        if(verbose_)
          PRINT_WARN("'%s' has stopRequest", Base::name().c_str());
        Base::stopRequest(time);
      }

      if(server_->isActive())
      {
        // active goal, update computation
        return Base::updateRequest(time, period);
      }
      return true;
    }

  protected:
    virtual bool onPreemptRequest() { return true; };

  private:
    void goalCallback()
    {
      // got a goal, start the derived object
      if(!Base::isRunning())
      {
        if(verbose_)
          PRINT_WARN("'%s' has new goal", Base::name().c_str());
        goal_ = *server_->acceptNewGoal();
      }
      else
      {
        PRINT_WARN("'%s' already running", Base::name().c_str());
      }
    }

    void preemptCallback()
    {
      if(onPreemptRequest())
      {
        if(verbose_)
          PRINT_WARN("'%s' has preemt request", Base::name().c_str());
        server_->setPreempted();
      }
    }

  };

}

#endif