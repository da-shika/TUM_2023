/*! \file
 *
 * \author Simon Armleder
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#ifndef CONTROL_CORE_STATE_STATE_MACHINE_H_
#define CONTROL_CORE_STATE_STATE_MACHINE_H_

#include <control_core/statemachine/state_base.h>
#include <control_core/interfaces/controller_base.h>

#include <unordered_map>

namespace cc 
{

  /**
   * @brief EnumClassHash needed for c++11
   */
  struct EnumClassHash
  {
    template <typename T>
    std::size_t operator()(T t) const
    {
      return static_cast<std::size_t>(t);
    }
  };

  /**
   * @brief StateMachineBase Class
   * 
   * A simple template based statemachine class.
   * 
   * The first state added is used as inital state and set as active.
   * All other added states are inactive.
   * Multiple states can be set active at the same time.
   * 
   * A reference to the statemachine is added to each state when calling add()
   * and used to to get the context from the statemachine.
   * 
   * Note: events are not saved and only handled by the currently active state
   * 
   */
  template<typename _Derived, typename _Context, typename _Event, typename _Id>
  class StateMachineBase : public cc::ModuleBase
  {
  public:
    typedef cc::ModuleBase Base;
    typedef _Derived Derived;
    typedef _Context Context;
    typedef _Event Event;
    typedef _Id Id;

    typedef std::shared_ptr<StateBase<Derived> > StatePtr;
    typedef std::unordered_map<Id, StatePtr, EnumClassHash> StateMap;
    typedef std::vector<Id> IdVector;

  private:
    bool verbose_;          // print information

  protected:
    Context context_;       // holds context variables shared between states

    StateMap states_;       // all states
    StateMap active_;       // all active state 

    IdVector start_ids_;    // ids of states that are started
    IdVector stop_ids_;     // ids of states that are stoped

  public:
    /**
     * @brief Construct a new State Machine object
     * 
     * @param name 
     */
    StateMachineBase(const std::string& name, bool verbose=false) : 
      Base(name),
      verbose_(verbose)
    {
    }

    /**
     * @brief Destroy the State Machine object
     * 
     */
    virtual ~StateMachineBase()
    {
    }

    Derived& derived()
    {
      return *static_cast<Derived*>(this);
    }

    /**
     * @brief Access to the statemachine context
     * 
     * @return Context& 
     */
    Context& context()
    {
      return context_;
    }

    /**
     * @brief active states set
     * 
     * @return const StateMap& 
     */
    bool is(const Id& id) const
    {
      return active_.find(id) != active_.end();
    }

    /**
     * @brief add new state
     * 
     * Note, the first state that is added to the statemachine is automatically
     * set to be active. Default: new states are inactive.
     * 
     * @param state the state
     * @param active sets as active or inactive
     * @return true 
     * @return false 
     */
    bool add(StatePtr state, bool active = false)
    {
      states_[state->id()] = state;
      state->setStateMachine(derived());
      if(active_.empty())
      {
        // allways insert first element
        active_[state->id()] = state;
      }
      else if(active)
      {
        // insert active elements
        active_[state->id()] = state;
      }
      if(verbose_)
        PRINT_WARN("'%s': State: '%s'", Base::name().c_str(), state->name().c_str());

      return true;
    }

    virtual bool init(ros::NodeHandle& nh, cc::Parameters& global_params) override
    {
      if(states_.empty())
      {
        PRINT_ERROR("'%s': No states added to sm.", Base::name().c_str());
        return false;
      }
      for(auto state : states_)
      {
        if(!state.second->initRequest(nh, global_params))
        {
          PRINT_ERROR("'%s': Failed to init state: '%s'", Base::name().c_str(), state.second->name().c_str());
          return false;
        }
      }
      return true;
    }

    virtual void start(const ros::Time &time) override
    {
      // set all active state into running
      for(auto state : active_)
      {
        state.second->startRequest(time, Id());
        if(verbose_)
          PRINT_WARN("'%s': State: '%s'", Base::name().c_str(), state.second->name().c_str());
      }
    }

    virtual bool update(const ros::Time &time, const ros::Duration &period) override
    {
      // update states
      bool ok = true;
      for(auto state : states_)
      {
        if(state.second->isRunning())
          ok &= state.second->updateRequest(time, period);
      }

      // apply changes
      applyChange(time);

      return ok;
    }

    virtual void stop(const ros::Time &time) override
    {
      // force all active states to stop
      for(auto state : active_)
      {
        state.second->stopRequest(time, Id());
        if(verbose_)
          PRINT_ERROR("'%s': State: '%s'", Base::name().c_str(), state.second->name().c_str());
      }
    }

    virtual void handle(const Event& e)
    {
      // dispatch event to active states
      for(auto state : active_)
        state.second->handle(e);
    }

    /**
     * @brief call this function to change the state from current to target
     * 
     * @param time 
     * @param current 
     * @param target 
     * @return true 
     * @return false 
     */
    bool change(
      const ros::Time &time, 
      const Id& current,
      const Id& target)
    {
      if(target == Id())
      {
        // no stepping required, empty id means stay
        return false;
      }
      if(target == current)
      {
        // same state as current, just stay
        return false;
      }

      // save ids
      start_ids_.push_back(target);
      stop_ids_.push_back(current);
      return true;
    }

    /**
     * @brief change the state form current to multiple target states
     * 
     * @param time 
     * @param current 
     * @param targets 
     * @return true 
     * @return false 
     */
    bool change(
      const ros::Time &time, 
      const Id& current,
      const std::vector<Id>& targets)
    {
      bool ok = true;
      for(auto target : targets)
      {
        ok &= change(time, target);
      }
      return ok;
    }

    /**
     * @brief check if state is part of this statemachine
     * 
     */
    StatePtr has(const Id& id)
    {
      auto elem = states_.find(id);
      if(elem == states_.end())
      {
        return nullptr;
      }
      return elem->second;
    }

  private:

    bool applyChange(const ros::Time& time)
    {
      for(size_t i = 0; i < start_ids_.size(); ++i)
      {
        const Id cur_id = stop_ids_[i];
        const Id next_id = start_ids_[i];

        // find state to be started
        StatePtr next = has(next_id);
        if(!next)
        {
          PRINT_ERROR("'%s' has no state with given id", Base::name().c_str());
          return false;
        }

        // find state to be stoped and stop it if active
        typename StateMap::iterator cur_it = active_.find(cur_id);
        if(cur_it != active_.end())
        {
          // stop state
          StatePtr cur = cur_it->second;
          cur->stopRequest(time, next_id);
          active_.erase(cur_it);

          if(verbose_)
            PRINT_WARN("change '%s' -> '%s'", cur->name().c_str(), next->name().c_str());
        }

        // finally start the next state
        next->startRequest(time, cur_id);
        active_[next_id] = next;
      }

      // clear lists
      start_ids_.clear();
      stop_ids_.clear();
      return true;
    }

  };
}

#endif