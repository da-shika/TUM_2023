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

#ifndef CONTROL_CORE_TINY_STATE_MACHINE_H_
#define CONTROL_CORE_TINY_STATE_MACHINE_H_

#include <vector>
#include <iostream>

namespace cc 
{

  /**
   * @brief Tiny Statemachine Class 
   * 
   * Used to define minimal statemachines within a single class.
   * Usage: Derive this class set the transition table via addTransitions()
   * in the constructor. 
   * 
   * Note: StateId and EventId should be enums and their size must be given to
   * the StateMachine to alocate the transiton table.
   * 
   * @tparam Derived Derived Class
   * @tparam StateId State Enum
   * @tparam EventId Event Enum
   */
  template <typename Derived, typename StateId, typename EventId>
  class StateMachine
  {
  public:
    using TransitionFunction = void (Derived::*)();
    struct Row {StateId from; StateId to; EventId event; TransitionFunction func; 
      Row(StateId from, StateId to, EventId event, TransitionFunction func=nullptr) : 
        from(from), to(to), event(event), func(func) {} };
    using Rows = std::vector<Row>;

  private:
    std::vector<std::vector<std::pair<StateId, TransitionFunction>>> tabel_;
    StateId cur_;
    size_t n_states_;
    size_t n_events_;

  public:
    StateMachine(const StateId& state, size_t n_states, size_t n_events) : 
      cur_(state), n_states_(n_states), n_events_(n_events)
    {
      tabel_.resize(n_states);
      for (size_t i = 0; i < n_states_; i++) {
        tabel_[i].resize(n_events);
        for (size_t j = 0; j < n_events; j++) {
          tabel_[i][j] = std::make_pair(static_cast<StateId>(n_states), nullptr);
        }
      }
    }

    bool is(const StateId& state) const
    { 
      return cur_==state;
    };

    const StateId& state() const
    { 
      return cur_;
    }

    void reset(const StateId& state)
    {
      cur_ = state;
    }

    void add(const StateId& from, const StateId& to, const EventId& event, TransitionFunction func = nullptr) 
    {
      tabel_[from][event] = std::make_pair(to, func);
    }

    void add(Rows rows)
    {
      for(auto& row : rows)
        add(row.from, row.to, row.event, row.func);
    }

    void handle(const EventId& event) 
    {
      const StateId& next = tabel_[cur_][event].first;
      if (next != n_states_) 
      {
        TransitionFunction func = tabel_[cur_][event].second;
        cur_ = next;
        if (func) (derived().*func)();
      }
    }

    Derived& derived()
    {
      return *static_cast<Derived*>(this);
    }
  };
}

#endif