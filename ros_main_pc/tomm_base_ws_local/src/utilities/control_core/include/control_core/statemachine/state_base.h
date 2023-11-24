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

#ifndef CONTROL_CORE_STATE_MACHINE_STATE_BASE_H_
#define CONTROL_CORE_STATE_MACHINE_STATE_BASE_H_

#include <control_core/interfaces/generic_module_base.h>

namespace cc 
{
  /**
   * @brief StateBase Class
   * 
   * State requires init(), start(), update() and handle() function to be overloaded.
   * Every state is identified through its name. Transitions are handled
   * by calling the change function with the target state name.
   * 
   * StateMachine is the Type of the Parentstatemachine
   * Context is the Type that holds shared variables
   * Event is the event type
   * 
   */
  template<typename _StateMachine>
  class StateBase : public cc::GenericModuleBase
  {
    public:
      typedef cc::GenericModuleBase Base;
      typedef _StateMachine StateMachine;
      typedef typename StateMachine::Context Context;
      typedef typename StateMachine::Event Event;
      typedef typename StateMachine::Id Id;

    private:
      Id id_;                       // identifier for this state
      StateMachine* sm_;            // pointer to parent state machine

    public:
      /**
       * @brief Construct a new State Base object
       * 
       * @param name 
       */
      StateBase(const std::string& name, const Id& id) : 
        Base{name},
        id_{id},
        sm_{nullptr}
      {
      }

      /**
       * @brief Destroy the State Base object
       * 
       */
      virtual ~StateBase()
      {
      }

      /**
       * @brief start the state
       * 
       */
      bool startRequest(const ros::Time &time, const Id& prev = Id())
      {
        if (!Base::moduleSetStartState(ros::Time::now()))
        {
          ROS_ERROR("State::startRequest() on '%s' called but not initalized.", Base::name().c_str());
          return false;
        }
        start(time, prev);
        return true;
      }

      /**
       * @brief stop the state
       * 
       * @param time 
       * @param next 
       * @return true 
       * @return false 
       */
      bool stopRequest(const ros::Time &time, const Id& next = Id())
      {
        if (Base::moduleSetStopState(ros::Time::now()))
        {
          stop(time, next);
          return true;
        }
        return false;
      }

      /**
       * @brief update the state
       * 
       * returns the identifier of the next state and and empty string if
       * no change of state is required.
       * 
       */
      virtual bool updateRequest(const ros::Time &time, const ros::Duration &period)
      {
        if (!Base::isRunning())
        {
          ROS_ERROR("State::updateRequest() on '%s' called but not running.", Base::name().c_str());
          return false;
        }
        return update(time, period);
      }

      /**
       * @brief handle an incomming event
       * 
       * returns the identifier of the next state and and empty string if
       * no change of state is required.
       * 
       */
      virtual void handle(const Event& e) { };

      /**
       * @brief set the statemachine (used during add())
       * 
       * @param sm 
       */
      void setStateMachine(StateMachine& sm) { sm_ = &sm; }

      /**
       * @brief get the id of this state
       * 
       */
      const Id & id() { return id_; }

    protected:

      /**
       * @brief change to the next state
       * 
       * @param time 
       * @param target 
       * @return true 
       * @return false 
       */
      bool change(
        const ros::Time &time, 
        const Id& target)
      {
        if(sm_)
          return sm_->change(time, id_, target);
        return false;
      }

      /**
       * @brief change to the next states
       * 
       * Multiple states can be active simultaniously
       * 
       * @param time 
       * @param targets 
       * @return true 
       * @return false 
       */
      bool change(
        const ros::Time &time, 
        const std::vector<Id>& targets)
      {
        if(sm_)
          return sm_->change(time, id_, targets);
        return false;
      }

      /**
       * @brief Access to the statemachine ctx
       * 
       * @return Context& 
       */
      const Context& ctx() const
      {
        assert(("Statemachine ptr must be set", sm_ != nullptr));
        return sm_->context();
      }
      Context& ctx()
      {
        assert(("Statemachine ptr must be set", sm_ != nullptr));
        return sm_->context();
      }

      /**
       * @brief access to state machine
       * 
       * @return const StateMachine* 
       */
      const StateMachine* stateMachine() const
      {
        return sm_;
      }

      /**
       * @brief Get the statemachine name
       * 
       * @param time 
       * @param prev 
       */
      const std::string& stateMachineName()
      {
        static std::string default_name;
        if(sm_)
          return sm_->name();
        return default_name;
      }

      /**
       * @brief called when state is started
       * 
       * @param time 
       * @param prev 
       */
      virtual void start(
        const ros::Time &time,
        const Id& prev) {}

      /**
       * @brief called periodically for update
       * 
       * @param time 
       * @param period 
       * @return true 
       * @return false 
       */
      virtual bool update(
        const ros::Time &time,
        const ros::Duration &period) = 0;

      /**
       * @brief called when state is stoped
       * 
       * @param time 
       * @param next 
       */
      virtual void stop(
        const ros::Time &time,
        const Id& next) {}
  };

}

#endif