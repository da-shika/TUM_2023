#include <control_core/systems/mass_spring_damper.h>

namespace cc
{  
  MassSpringDamperSystem::MassSpringDamperSystem() : 
    mass_(cc::Vector6::Zero()),
    spring_(cc::Vector6::Zero()),
    damping_(cc::Vector6::Zero()),
    acc_lb_(-1e10*cc::Vector6::Ones()),
    acc_ub_(1e10*cc::Vector6::Ones()),
    vel_lb_(-1e10*cc::Vector6::Ones()),
    vel_ub_(1e10*cc::Vector6::Ones()),
    pos_lb_(-1e10*cc::LinearPosition::Ones()),
    pos_ub_(1e10*cc::LinearPosition::Ones())
  {
  }

  MassSpringDamperSystem::~MassSpringDamperSystem()
  {
  }

  bool MassSpringDamperSystem::init(cc::Parameters& params)
  {
    params.get("mass", mass_);
    params.get("spring", spring_);
    params.get("damping", damping_);
    if(!params.get("acc_lb", acc_lb_) || 
       !params.get("acc_ub", acc_ub_) ||
       !params.get("vel_lb", vel_lb_) ||
       !params.get("vel_ub", vel_ub_) ||
       !params.get("pos_lb", pos_lb_) || 
       !params.get("pos_ub", pos_ub_))
    {
      ROS_WARN("MassSpringDamperSystem::init: Using default limits");
    }
    return true;
  }

  void MassSpringDamperSystem::setMass(const cc::Vector6& mass)
  {
    mass_ = mass;
  }

  void MassSpringDamperSystem::setSpring(const cc::Vector6& spring)
  {
    spring_ = spring;
  }

  void MassSpringDamperSystem::setDamping(const cc::Vector6& damping)
  {
    damping_ = damping;
  }

  void MassSpringDamperSystem::setAccBounds(const cc::Vector6& acc_lb, const cc::Vector6& acc_ub)
  {
    acc_lb_ = acc_lb;
    acc_ub_ = acc_ub;
  }

  void MassSpringDamperSystem::setVelBounds(const cc::Vector6& vel_lb, const cc::Vector6& vel_ub)
  {
    vel_lb_ = vel_lb;
    vel_ub_ = vel_ub;
  }

  void MassSpringDamperSystem::setPosBounds(const cc::LinearPosition& pos_lb, const cc::LinearPosition& pos_ub)
  {
    pos_lb_ = pos_lb;
    pos_ub_ = pos_ub;
  }

  void MassSpringDamperSystem::reset(const cc::CartesianPosition inital_pos)
  {
    state_.pos() = inital_pos;
    state_.vel().setZero();
    state_.acc().setZero();
  }

  void MassSpringDamperSystem::update(const cc::Wrench wrench, const ros::Duration& dt)
  {
    // system in local frame
    state_.acc() = 
     mass_.cwiseInverse().cwiseProduct(
        spring_.cwiseProduct(wrench) - damping_.cwiseProduct(state_.vel()));
    cc::clamp(state_.acc(), acc_lb_, acc_ub_);
    
    // integrate to position and acceleration
    cc::integrateStateBody(state_, state_, dt.toSec());
    state_.pos().angular().normalize();

    // vel
    cc::Vector6 v_mean = state_.vel() + 0.5 * dt.toSec() * state_.acc();
    state_.vel() = state_.vel() + dt.toSec() * state_.acc();
    for(size_t i = 0; i < state_.vel().size(); ++i)
    {
      if(state_.vel()[i] > vel_ub_[i])
      {
        state_.acc()[i] = 0.0;
        state_.vel()[i] = vel_ub_[i];
        v_mean[i] = vel_ub_[i];
      }
      else if(state_.vel()[i] < vel_lb_[i])
      {
        state_.acc()[i] = 0.0;
        state_.vel()[i] = vel_lb_[i];
        v_mean[i] = vel_lb_[i];
      }
    }

    // pos
    state_.pos() = cc::integrateVelocityBody(state_.pos(), v_mean, dt.toSec());
    for(size_t i = 0; i < 3; ++i)
    {
      if(state_.pos().linear()[i] > pos_ub_[i])
      {
        state_.pos().linear()[i] = pos_ub_[i];
        state_.vel().linear()[i] = 0.0;
        state_.acc().linear()[i] = 0.0;
      }
      else if(state_.pos().linear()[i] < pos_lb_[i])
      {
        state_.pos().linear()[i] = pos_lb_[i];
        state_.vel().linear()[i] = 0.0;
        state_.acc().linear()[i] = 0.0;
      }
    }
  }

  const cc::CartesianState& MassSpringDamperSystem::state() const
  {
    return state_;
  }
}