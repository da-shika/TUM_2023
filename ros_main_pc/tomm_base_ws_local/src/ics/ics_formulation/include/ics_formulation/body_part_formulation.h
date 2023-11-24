#ifndef ICS_FORMULATION_BODY_PART_FORMULATION_H_
#define ICS_FORMULATION_BODY_PART_FORMULATION_H_

#include <ics_tsid_wrapper/tsid_wrapper.h>

namespace ics
{

  /**
   * @brief BodyPartFormulation Class
   * 
   * Formulation for a robot body part (e.g Hands, Feet, ect.)
   * A BodyPart has a Motion (SE3Equality) and Contact (Contact6dExt) task.
   * It can be in states:
   *  Free: No task active on this body
   *  Motion: Motion task active
   *  Contact: Contact task active
   * 
   * Note: If the contact task is not specified we not allow contact
   * activation of this frame.
   * 
   */
  class BodyPartFormulation : public cc::ModuleBase
  {
    public:
      typedef ModuleBase Base;
      enum State {FREE, MOTION, CONTACT};

    public:
      typedef tsid::trajectories::TrajectorySample TrajectorySample;
      typedef std::shared_ptr<tsid::contacts::Contact6dExt> ContactTaskPtr;
      typedef std::shared_ptr<tsid::tasks::TaskSE3Equality> MotionTaskPtr;

    private:
      ics::TSIDWrapper& tsid_;                  //!< idyn formulation

      State state_;                             //!< limb state

      std::string motion_name_;                 //!< motion task name
      std::string contact_name_;                //!< contact task name

      ContactTaskPtr contact_task_;             //!< contact taks ptr
      MotionTaskPtr motion_task_;               //!< motion task ptr

    public:
      BodyPartFormulation(ics::TSIDWrapper& tsid, const std::string& name);
      virtual ~BodyPartFormulation();      

      bool free() const { return state_ == FREE; }
      bool inContact() const { return state_ == CONTACT; }
      bool inMotion() const { return state_ == MOTION; }

      pinocchio::FrameIndex frameId() const { return motion_task_->frame_id(); }

      const std::string& frameName() const { 
        return tsid_.robot().frameName(motion_task_->frame_id());}

      //////////////////////////////////////////////////////////////////////////
      // motion task
      //////////////////////////////////////////////////////////////////////////

      bool addMotion();
      bool removeMotion();

      void setMotionReference(TrajectorySample &ref);
      void setMotionReference(const cc::CartesianPosition &ref);
      void setMotionReference(const cc::CartesianState &ref);

      cc::CartesianState motionReference();
      cc::CartesianPosition motionPosition();
      cc::CartesianState motionState();

      bool updateMotionWeight(cc::Scalar weight);
      bool updateTaskGains(const cc::VectorX &Kp, const cc::VectorX &Kd = cc::VectorX());
      void udpateMotionMask(const cc::VectorX &mask);

      //////////////////////////////////////////////////////////////////////////
      // contact task
      //////////////////////////////////////////////////////////////////////////

      bool addContact(const cc::CartesianPosition &ref = cc::CartesianPosition::Identity(), cc::Scalar transition_time = 0);
      bool removeContact(cc::Scalar transition_time = 0);

      bool setForceReference(const cc::Wrench& wrench);

      bool updateRigidContactWeights(cc::Scalar force_regularization_weight, cc::Scalar motion_weight = -1.0);
      bool updateContactGains(const cc::Vector6 &Kp, const cc::VectorX &Kd = cc::VectorX());

      cc::Wrench contactWrench();
      cc::Scalar contactNormalForce();

      //////////////////////////////////////////////////////////////////////////
      // direct access
      //////////////////////////////////////////////////////////////////////////
      MotionTaskPtr motion() { return motion_task_; }
      ContactTaskPtr contact() { return contact_task_; }

    protected:
      virtual bool init(ros::NodeHandle &nh, Base::Parameters &global_params) override;
  };

}

#endif