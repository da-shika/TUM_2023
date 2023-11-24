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

#ifndef WALKING_CORE_FOOT_STEP_H_
#define WALKING_CORE_FOOT_STEP_H_

#include <control_core/types/contact.h>
#include <control_core/types/body_id.h>
#include <walking_core_msgs/FootStep.h>

namespace control_core 
{

  template<typename _Scalar>
  class FootStep
  {
  public:
    typedef _Scalar Scalar;
    typedef Contact<Scalar> ContactType;

    /*!
    * \brief Construct as Zero.
    */
    static const FootStep& Zero()
    {
      static FootStep v;
      static bool once = false;
      if(!once)
      {
        v.setZero();
        once = true;
      }
      return v;
    }

  private:
    ContactType contact_;
    BodyId body_id_;
    size_t n_step_;
    bool final_step_;

  public:
    FootStep()
    {
    }

    FootStep(const FootStep& other) : 
      contact_(other.contact_),
      body_id_(other.body_id_),
      n_step_(other.n_step_),
      final_step_(other.final_step_)
    {
    }

    FootStep(const ContactType& contact, const BodyId& body_id, size_t n_step = 0, bool final_step=false) :
      contact_(contact),
      body_id_(body_id),
      n_step_(n_step),
      final_step_(final_step)
    {
    }

    ContactType& contact()
    {
      return contact_;
    }

    const ContactType& contact() const
    {
      return contact_;
    }

    BodyId& bodyId()
    {
      return body_id_;
    }

    const BodyId& bodyId() const
    {
      return body_id_;
    }

    bool& isFinalStep()
    {
      return final_step_;
    }

    const bool& isFinalStep() const
    {
      return final_step_;
    }

    size_t& nStep()
    {
      return n_step_;
    }

    size_t nStep() const
    {
      return n_step_;
    }

    /*!
    * \brief Assignment of Footstep
    */
    FootStep& operator=(const FootStep& other)
    {
      contact() = other.contact_;
      bodyId() = other.body_id_;
      nStep() = other.n_step_;
      isFinalStep() = other.final_step_;
      return *this;
    }

    /*!
    * \brief Assignment of walking_core_msgs::FootStep.
    */
    FootStep& operator=(const walking_core_msgs::FootStep& msg)
    {
      contact_ = msg.contact;
      body_id_ = BodyId(static_cast<BodyId::Value>(msg.body_id.data));
      final_step_ = msg.final_step.data;
      n_step_ = msg.n_step.data;
      return *this;
    }

    /*!
    * \brief Conversion to walking_core_msgs::FootStep.
    */
    operator walking_core_msgs::FootStep() const
    {
      walking_core_msgs::FootStep msg;
      msg.contact = contact_;
      msg.body_id.data = static_cast<int>(body_id_);
      msg.final_step.data = final_step_;
      msg.n_step.data = n_step_;
      return msg;
    }  

    /*!
    * \brief Conversion to walking_core_msgs::FootStep.
    */
    walking_core_msgs::FootStep toFootStepMsg() const
    {
      return static_cast<walking_core_msgs::FootStep>(*this);
    }

    /**
     * @brief set zero
     */
    void setZero()
    {
      contact().setZero();
      bodyId() = BodyId::LEFT_FOOT;
      nStep() = 0;
      isFinalStep() = false; 
    }

    std::string toString() const
    {
      std::stringstream ss;
      ss << "n_step=    " << nStep() << std::endl;
      ss << "body_id=   " << bodyId().toString() << std::endl;
      ss << "final_step=" << isFinalStep() << std::endl;
      ss << "contact=\n" << contact().toString() << std::endl;
      return ss.str();
    }

  };

}

#endif