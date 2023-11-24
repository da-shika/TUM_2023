#ifndef ICS_WHOLE_BODY_CONTROLLER_COMPOSITE_BEHAVIOR_H_
#define ICS_WHOLE_BODY_CONTROLLER_COMPOSITE_BEHAVIOR_H_

////////////////////////////////////////////////////////////////////////////////
// ics_behavior includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_behavior/behavior_base.h>

namespace ics
{

  class CompositeBehavior : public BehaviorBase
  {
  public:
    typedef BehaviorBase Base;
    typedef BehaviorBase::Base Module;
    typedef std::vector<std::shared_ptr<BehaviorBase> > Composites;

  private:
    Composites composites_;

  public:
    CompositeBehavior(const std::string& name, const Composites& composites) : 
      Base(name),
      composites_(composites)
    {
    }

    virtual ~CompositeBehavior()
    {
    }

    virtual bool initRequest(ros::NodeHandle& nh, Base::Parameters& params) override
    {
      bool ret = true;
      for(auto composit : composites_)
        ret &= composit->initRequest(nh, params);
      ret &= Module::initRequest(nh, params);
      return ret;
    }

    virtual void setController(Base::ControllerPtr controller)
    {
      for(auto composit : composites_)
        composit->setController(controller);
    }

    virtual bool loadRequest()
    {
      if(!Module::isInitialized())
      {
        PRINT_ERROR("'%s' Not initalized", Base::name().c_str());
        return false;
      }

      bool ret = true;
      for(auto composit : composites_)
        ret &= composit->loadRequest();

      is_loaded_ = true;
      return ret;
    }

    virtual bool startRequest(const ros::Time &time) override
    {
      if (!is_loaded_ || !Module::moduleSetStartState(ros::Time::now()))
      {
        PRINT_ERROR("'%s' called but not initalized.", Module::name().c_str());
        return false;
      }
      has_updated_once_ = false;

      bool ret = true;
      for(auto composit : composites_)
        ret &= composit->startRequest(time);
      return ret;
    }

    virtual bool updateRequest(const ros::Time &time, const ros::Duration &period) override
    {
      if (!Module::isRunning())
      {
        PRINT_ERROR("'%s' called but not running.", Module::name().c_str());
        return false;
      }

      bool ret = true;
      for(auto composit : composites_)
        ret &= composit->updateRequest(time, period);

      has_updated_once_ = true;
      return ret;
    }

    virtual bool stopRequest(const ros::Time &time) override
    {
      if (Module::moduleSetStopState(ros::Time::now()))
      {
        bool ret = true;
        for(auto composit : composites_)
          ret &= composit->stopRequest(time);
        return ret;
      }
      return false;
    }
    
    virtual bool unloadRequest() override
    {
      if(Module::isRunning())
      {
        PRINT_ERROR("'%s' called but still running.", Module::name().c_str());
        return false;
      }

      if(!is_loaded_)
      {
        PRINT_WARN("'%s' is already unloaded", Module::name().c_str());
        return true;
      }

      bool ret = true;
      for(auto composit : composites_)
        ret &= composit->unloadRequest();

      is_loaded_ = false;
      return ret;
    }
    
    virtual void publishRequest(const ros::Time &time) override
    {
      if(has_updated_once_)
      {
        for(auto composit : composites_)
          composit->publishRequest(time);
      }
    }

  protected:
    virtual bool update(
      const ros::Time &time,
      const ros::Duration &period) override
    {
      // nothing to do
      return true;
    }

    virtual bool init(ros::NodeHandle& nh, Parameters& params) override
    {
      // nothing to do
      return true;
    }

  };
}

#endif