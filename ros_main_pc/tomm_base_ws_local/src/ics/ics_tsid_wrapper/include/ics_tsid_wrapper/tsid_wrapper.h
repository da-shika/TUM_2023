#ifndef ICS_TSID_SOT_TSID_H_
#define ICS_TSID_SOT_TSID_H_

////////////////////////////////////////////////////////////////////////////////
// Using TSID with ProxSuite
////////////////////////////////////////////////////////////////////////////////
// #define TSID_WITH_PROXSUITE

////////////////////////////////////////////////////////////////////////////////
// Using Pinocchio with Fast collision lib
////////////////////////////////////////////////////////////////////////////////
#define PINOCCHIO_WITH_HPP_FCL

////////////////////////////////////////////////////////////////////////////////
// tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/math/fwd.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/robots/fwd.hpp>

////////////////////////////////////////////////////////////////////////////////
// control_core includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/interfaces/module_base.h>

////////////////////////////////////////////////////////////////////////////////
// ics_tsid includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_tsid_common/utilities/tasks.h>
#include <ics_tsid_common/interfaces/tsid_wrapper_interface.h>
#include <ics_tsid_wrapper/utilities.h>

////////////////////////////////////////////////////////////////////////////////
// ics_robot includes
////////////////////////////////////////////////////////////////////////////////
#include <ics_robot_wrapper/robot_wrapper.h>

namespace ics
{
  /**
   * @brief Holds the TaskSpaceInverseDynamic Formulation
   * 
   * Note: this class also holds the geometry data used for collision
   * computation.
   *
   */
  class TSIDWrapper : 
    public cc::ModuleBase,
    public TSIDWrapperInterface
  {
  public:
    typedef cc::ModuleBase Base;
    typedef TSIDWrapperInterface Interface;

    typedef tsid::InverseDynamicsFormulationAccForce Formulation;
    typedef tsid::solvers::SolverHQPBase Solver;
    typedef Formulation::Data Data;
    typedef pinocchio::GeometryData GeometryData;
    typedef RobotWrapper Robot;

    typedef std::shared_ptr<tsid::tasks::TaskBase> TaskPtr;
    typedef std::shared_ptr<tsid::contacts::Contact6dExt> ContactPtr;

    typedef tsid::trajectories::TrajectorySample TrajectorySample;
    typedef std::shared_ptr<tsid::tasks::TaskSE3Equality> TaskSE3EqualityPtr;

    typedef std::unordered_map<std::string, TaskWeightSpline> TaskWeightSplineMap;

  protected:
    bool verbose_;                    // print messages
    bool use_close_loop_;             // use real state feedback or not
    bool has_been_solved_;            // has been solved once

    std::int64_t solver_dur_;           // time for solving [ns]
    std::int64_t geometry_dur_;   // time for self collision checks

    //////////////////////////////////////////////////////////////////////////
    // robot
    //////////////////////////////////////////////////////////////////////////
    Robot &robot_;                    // robot
    GeometryData geometry_data_;      // robot geometry data 

    //////////////////////////////////////////////////////////////////////////
    // tsid algorithms
    //////////////////////////////////////////////////////////////////////////

    std::unique_ptr<Formulation> formulation_; // formulation
    std::unique_ptr<Solver> solver_;           // solver

    //////////////////////////////////////////////////////////////////////////
    // tsid variables
    //////////////////////////////////////////////////////////////////////////

    int na_; // size of actuated part
    int nq_; // size of q vector
    int nv_; // size of v vector

    ics::TaskMap tasks_;                  // all loaded motion tasks
    ics::TaskMap active_tasks_;           // all active tasks

    ics::ContactMap contacts_;           // all loaded contact tasks
    ics::WeightMap weights_;             // all task/contact weights
    ics::ContactVector active_contacts_; // currently active contacts

    std::vector<std::string> active_contact_names_; // active contact names
    std::vector<std::string> all_contact_names_;    // all contact names

    cc::Scalar dt_ref_; // reference sample period
    cc::Scalar t_;      // solver time

    tsid::solvers::HQPOutput solution_; // solution
    cc::VectorX a_sol_;                 // solution joint accelerations                             

    //////////////////////////////////////////////////////////////////////////
    // virtual robot state variables
    //////////////////////////////////////////////////////////////////////////
    
    cc::VectorX q_;        // tsid joint positions
    cc::VectorX v_;        // tsid joint velocities

    //////////////////////////////////////////////////////////////////////////
    // spline 
    //////////////////////////////////////////////////////////////////////////

    TaskWeightSplineMap task_weight_spline_map_;

  public:
    TSIDWrapper(Robot &robot, const std::string &name);

    virtual ~TSIDWrapper();

    ////////////////////////////////////////////////////////////////////////////
    // managing of tasks
    ////////////////////////////////////////////////////////////////////////////

    /**
     * @brief load a task and optionally activate it
     */
    virtual bool loadTask(const std::string& name, TaskPtr task, cc::Scalar weight = -1, unsigned int priority = 1, bool activate=true) override;

    /**
     * @brief adds a loaded task into the formulation
     *
     * Note: Task must already be loaded first (see loadTask)
     */
    bool addTask(const std::string &name, cc::Scalar weight = -1, unsigned int priority = 1) override;

    /**
     * @brief removes a loaded task from the formulation
     * 
     * Note: Task is removed from formulation, but remain in TaskMap (for reactivation)
     *
     * @param name
     * @param transition_time
     * @return true
     * @return false
     */
    bool removeTask(const std::string &name, cc::Scalar transition_time = 0) override;

    /**
     * @brief removes a task from the formulation and also from the taskmap
     */
    bool unloadTask(const std::string& name);

    ////////////////////////////////////////////////////////////////////////////
    // managing of contacts
    ////////////////////////////////////////////////////////////////////////////

    /**
     * @brief load contact, optionally activate it
     */
    virtual bool loadContact(const std::string &name, ContactPtr contact, cc::Scalar transition_time = 0, bool activate=true) override;

    /**
     * @brief adds a loaded contact into the formulation
     *
     * Note: Task must already be loaded
     */
    bool addContact(const std::string &name, cc::Scalar transition_time = 0) override;

    /**
     * @brief removes a loaded contact from the formulation
     * 
     * Note: contact is removed from formulation, but remain in ContactMap (for reactivation)
     */
    bool removeContact(const std::string &name, cc::Scalar transition_time = 0) override;

    /**
     * @brief removes a task from the formulation and also from the contact map
     */
    bool unloadContact(const std::string &name);

    /**
     * @brief Set the Reference To Current object (perfect match)
     *
     */
    bool setAllReferencesToCurrent(bool verbose=true);

    //////////////////////////////////////////////////////////////////////////
    // general tasks
    //////////////////////////////////////////////////////////////////////////

    cc::Scalar weight(const std::string& name) const;

    bool transitTaskWeight(const std::string &name, cc::Scalar weight_end, const ros::Duration& dur);

    bool updateTaskWeight(const std::string &name, cc::Scalar weight);

    void udpateTaskMask(const std::string &task_name, const cc::VectorX &mask);

    template <typename T>
    std::shared_ptr<T> task(const std::string &name) { 
      return ics::get_task<T>(tasks_, name); }

    template <typename T>
    bool updateTaskGains(const std::string &name, const cc::VectorX &Kp, const cc::VectorX &Kd = cc::VectorX());

    //////////////////////////////////////////////////////////////////////////
    // SE3 tasks
    //////////////////////////////////////////////////////////////////////////

    void setSe3TaskReference(const pinocchio::SE3 &ref, const std::string &task_name);

    void setSe3TaskReference(TrajectorySample &sample, const std::string &task_name);

    TrajectorySample se3TaskReference(const std::string &task_name);

    pinocchio::SE3 se3TaskPosition(const std::string &task_name);

    TrajectorySample se3TaskState(const std::string &task_name);

    TaskSE3EqualityPtr se3Task(const std::string &name) {
      return ics::get_task<tsid::tasks::TaskSE3Equality>(tasks_, name);}

    TaskSE3EqualityPtr se3TaskFromFrame(pinocchio::FrameIndex id);
    TaskSE3EqualityPtr se3TaskFromFrame(const std::string &frame);

    //////////////////////////////////////////////////////////////////////////
    // Contact tasks
    //////////////////////////////////////////////////////////////////////////

    bool updateRigidContactWeights(const std::string &contact_name, cc::Scalar force_regularization_weight, cc::Scalar motion_weight = -1.0);

    bool updateContactGains(const std::string &name, const cc::Vector6 &Kp, const cc::VectorX &Kd = cc::VectorX());

    cc::Wrench contactWrenchSolution(const std::string &contact_name);

    cc::Scalar contactNormalForceSolution(const std::string &contact_name);

    ContactPtr contactTask(const std::string &name) {
      return ics::get_contact(contacts_, name);
    }

    ContactPtr contactFromFrame(pinocchio::FrameIndex id);
    ContactPtr contactFromFrame(const std::string &frame);

    //////////////////////////////////////////////////////////////////////////
    // Update and solve
    //////////////////////////////////////////////////////////////////////////

    /**
     * @brief overrides the current state used by tsid
     *
     * Note this completly replaces virtual (integrated state). Jumps will
     * direclty effect the real robot.
     * 
     * Recompute will update the tsid data containter 
     * (only needed if solver is not called afterwards)
     *
     */
    virtual void setState(const cc::VectorX& q, const cc::VectorX& v, bool recompute=false);

    /**
     * @brief update the solver and robot using state input (q,v)
     * 
     * note: this function solves the hqp and integrates the robot state.
     * Updates the geometry placements.
     *
     * @param robot_state
     * @param time
     * @param period
     * @return cc::JointState
     */
    virtual bool update(
      const cc::VectorX& q, const cc::VectorX& v,
      const ros::Time &time, const ros::Duration &period);

    /**
     * @brief resize the solver if prev sizes differ from new ones
     * call this if new tasks are loaded into the formuatlion.
     *
     * @param verbose
     * @return true
     * @return false
     */
    bool resizeSolver(bool verbose);

    //////////////////////////////////////////////////////////////////////////
    // getter/ properties
    //////////////////////////////////////////////////////////////////////////

    /**
     * @brief check if contact in set of active contacts
     *
     * @param name
     * @return true
     * @return false
     */
    bool hasActiveContact(const std::string &name) const { return cc::has(active_contact_names_, name); }

    /**
     * @brief access to vector of active contacts
     *
     * @return const tasks::ContactVector&
     */
    const ics::ContactVector &activeContacts() const { return active_contacts_; }

    //////////////////////////////////////////////////////////////////////////
    // interface functions
    //////////////////////////////////////////////////////////////////////////

    /**
     * @brief check if task in the formulation
     */
    virtual bool hasTask(const std::string &name) const override { 
      return cc::has(tasks_, name); }

    /**
     * @brief check if contact in the formulation
     */
    virtual bool hasContact(const std::string &name) const override { 
      return cc::has(contacts_, name); }

    /**
     * @brief access to robot configuration
     */
    const cc::VectorX &q() const override { return q_; }

    /**
     * @brief access to generalized velocity
     */
    const cc::VectorX &v() const override { return v_; }

    /**
     * @brief access to generalized accelerations
     */
    const cc::VectorX &a() const override { return a_sol_; }

    /**
     * @brief access to tsid data container
     */
    Data &data() override { return formulation_->data(); }
    const Data &data() const override { return formulation_->data(); }

    /**
     * @brief access to the formulation
     *
     */
    Formulation &formulation() override { return *formulation_; }
    const Formulation &formulation() const override { return *formulation_; }

    /**
     * @brief access to the formulation
     *
     */
    Solver &solver() override { return *solver_; }
    const Solver &solver() const override { return *solver_; }
    const std::int64_t &solverDuration() const override { return solver_dur_; };

    /**
     * @brief access to the robot interface used by tsid
     * 
     */
    ics::RobotWrapperInterface &robotInterface() override { return robot_; };
    const ics::RobotWrapperInterface &robotInterface() const override { return robot_; };

    /**
     * @brief access to the robot
     */
    Robot &robot() { return robot_; };
    const Robot &robot() const { return robot_; };

    /**
     * @brief access to pinocchio geometry data
     * 
     * @return pinocchio::GeometryData& 
     */
    virtual pinocchio::GeometryData& geometryData() override { return geometry_data_; }
    virtual const pinocchio::GeometryData& geometryData() const override { return geometry_data_; }
    virtual const std::int64_t& geometryDuration() const override { return geometry_dur_; }

    /**
     * @brief print status
     */
    virtual std::string toString() const;

    /**
     * @brief reference sample period
     */
    cc::Scalar dtRef() const;

  //////////////////////////////////////////////////////////////////////////
  // private functions
  //////////////////////////////////////////////////////////////////////////

  protected:
    virtual bool init(ros::NodeHandle &nh, Base::Parameters &global_params) override;

  private:
    /**
     * @brief solve the hqp and store solution
     *
     * @param q
     * @param v
     * @param dt
     */
    bool solve(const cc::VectorX &q, const cc::VectorX &v, cc::Scalar dt);
    
    /**
     * @brief change task weight with spline function
     * 
     * @param dt 
     * @return true 
     * @return false 
     */
    void updateTaskWeightSplines(cc::Scalar dt);

    /**
     * @brief checks if task already in formulation
     * 
     * @param name 
     * @return true 
     * @return false 
     */
    bool isActiveTask(const TaskPtr& task) const;

  };
  
  /**
   * @brief templated updateTaskGains function 
   */
  template<typename T>
  bool TSIDWrapper::updateTaskGains(const std::string& name, const cc::VectorX& Kp, const cc::VectorX& Kd)
  {
    auto task = ics::get_task<T>(tasks_, name);
    if(!task)
    {
      PRINT_ERROR("'%s' does not exsits.", name.c_str());
      return false;
    }
    if(Kp.size() != task->dim())
    {
      PRINT_ERROR("'%s' has wrong gain size.", name.c_str());
      return false;
    }
    if(Kp.size() == Kd.size())
    {
      task->Kp(Kp);
      task->Kd(Kd);
    }
    else
    {
      task->Kp(Kp);
      task->Kd(2.0*Kp.cwiseSqrt());
    }
    return true;
  }

}

#endif