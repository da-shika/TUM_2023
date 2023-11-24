#include <ics_tsid_common/utilities/conversions.h>
#include <control_core/math.h>

namespace ics
{

  tsid::trajectories::TrajectorySample to_sample(const cc::VectorX& ref)
  {
    tsid::trajectories::TrajectorySample sample;
    sample.setValue(ref);
    sample.setDerivative(cc::VectorX::Zero(ref.size()));
    sample.setSecondDerivative(cc::VectorX::Zero(ref.size()));
    return sample;
  }

  tsid::trajectories::TrajectorySample to_sample(const pinocchio::SE3& ref)
  {
    tsid::trajectories::TrajectorySample sample;
    sample.resize(12, 6);
TSID_DISABLE_WARNING_PUSH
TSID_DISABLE_WARNING_DEPRECATED
    tsid::math::SE3ToVector(ref, sample.pos);
TSID_DISABLE_WARNING_POP
    return sample;
  }

  tsid::trajectories::TrajectorySample to_sample(const cc::JointState& ref)
  {
    tsid::trajectories::TrajectorySample sample;
TSID_DISABLE_WARNING_PUSH
TSID_DISABLE_WARNING_DEPRECATED
    sample.pos = ref.pos();
    sample.vel = ref.vel();
    sample.acc = ref.acc();
TSID_DISABLE_WARNING_POP
    return sample;
  }

  tsid::trajectories::TrajectorySample to_sample(const cc::LinearState& ref)
  {
    tsid::trajectories::TrajectorySample sample;
TSID_DISABLE_WARNING_PUSH
TSID_DISABLE_WARNING_DEPRECATED
    sample.pos = ref.pos();
    sample.vel = ref.vel();
    sample.acc = ref.acc();
TSID_DISABLE_WARNING_POP
    return sample;
  }

  tsid::trajectories::TrajectorySample to_sample(const cc::AngularState& ref)
  {
    typedef Eigen::Matrix<double,9,1> Vector9;
    tsid::trajectories::TrajectorySample sample(12, 6);
TSID_DISABLE_WARNING_PUSH
TSID_DISABLE_WARNING_DEPRECATED
    sample.pos.head(3).setZero();
    sample.vel.head(3).setZero();
    sample.acc.head(3).setZero();
    sample.pos.tail(9) = Eigen::Map<Vector9>(ref.pos().toRotationMatrix().data(), 9);
    sample.vel.tail(3) = ref.vel();
    sample.acc.tail(3) = ref.acc();
TSID_DISABLE_WARNING_POP
    return sample;
  }
  
  tsid::trajectories::TrajectorySample to_sample(const cc::CartesianPosition& ref)
  {
    typedef Eigen::Matrix<double,9,1> Vector9;
    tsid::trajectories::TrajectorySample sample(12, 6);
TSID_DISABLE_WARNING_PUSH
TSID_DISABLE_WARNING_DEPRECATED
    sample.pos.head(3) = ref.linear();
    sample.pos.tail(9) = Eigen::Map<Vector9>(ref.angular().toRotationMatrix().data(), 9);
    sample.vel.setZero();
    sample.acc.setZero();
TSID_DISABLE_WARNING_POP
    return sample;
  }

  tsid::trajectories::TrajectorySample to_sample(const cc::CartesianState& ref)
  {
    typedef Eigen::Matrix<double,9,1> Vector9;
    tsid::trajectories::TrajectorySample sample(12, 6);
TSID_DISABLE_WARNING_PUSH
TSID_DISABLE_WARNING_DEPRECATED
    sample.pos.head(3) = ref.pos().linear();
    sample.pos.tail(9) = Eigen::Map<Vector9>(ref.pos().angular().toRotationMatrix().data(), 9);
    sample.vel = ref.vel();
    sample.acc = ref.acc();
TSID_DISABLE_WARNING_POP
    return sample;
  }

  pinocchio::SE3 to_se3(const tsid::trajectories::TrajectorySample& ref)
  {
    const cc::VectorX& pos = ref.getValue();
    pinocchio::SE3 se3;
    se3.translation(pos.head<3>());
    se3.rotation(Eigen::Map<const Eigen::Matrix3d>(&pos(3), 3, 3));
    return se3;
  }

  pinocchio::SE3 to_se3(const cc::CartesianPosition& ref)
  {
    pinocchio::SE3 se3;
    se3.translation() = ref.linear();
    se3.rotation() = ref.angular().toRotationMatrix();
    return se3;
  }

  cc::CartesianPosition to_pose(const pinocchio::SE3& se3)
  {
    return cc::CartesianPosition(se3.rotation(), se3.translation());
  }
  
  cc::CartesianPosition to_pose(const tsid::trajectories::TrajectorySample& ref)
  {
    const cc::VectorX& pos = ref.getValue();
    return cc::CartesianPosition(
      Eigen::Map<const Eigen::Matrix3d>(&pos(3), 3, 3), pos.head(3));
  }

  cc::JointPosition to_joint_pos(const tsid::trajectories::TrajectorySample& ref)
  {
    cc::JointPosition q;
    q = ref.getValue();
    return q;
  }

  cc::HomogeneousTransformation to_transf(const pinocchio::SE3& se3)
  {
    return cc::HomogeneousTransformation(se3.rotation(), se3.translation());
  }

  cc::CartesianPosition to_transf(const tsid::trajectories::TrajectorySample& ref)
  {
    const cc::VectorX& pos = ref.getValue();
    return cc::HomogeneousTransformation(
      Eigen::Map<const Eigen::Matrix3d>(&pos(3), 3, 3), pos.head(3));
  }

  cc::LinearState to_linear_state(const tsid::trajectories::TrajectorySample& ref)
  {
    cc::LinearState state;
    state.pos() = ref.getValue();
    state.vel() = ref.getDerivative();
    state.acc() = ref.getSecondDerivative();
    return state;
  }

  cc::CartesianState to_cartesian_state(const tsid::trajectories::TrajectorySample& ref)
  {
    cc::CartesianState state;
    state.pos() = to_pose(ref);         
    state.vel() = ref.getDerivative();
    state.acc() = ref.getSecondDerivative();
    return state;
  }

  cc::JointState to_joint_state(const tsid::trajectories::TrajectorySample& ref)
  {
    cc::JointState state;
    state.pos() = ref.getValue();
    state.vel() = ref.getDerivative();
    state.acc() = ref.getSecondDerivative();
    return state;
  }

  std::string toString(const tsid::trajectories::TrajectorySample& sample)
  {
    std::ostringstream out;
    out << "pos    = [" << sample.getValue().transpose() << "]\n";
    out << "vel    = [" << sample.getDerivative().transpose() << "]\n";
    out << "acc    = [" << sample.getSecondDerivative().transpose() << "]\n";
    return out.str();
  }

  void to_tsid_state(const cc::RobotState& state, BaseType base_type, cc::VectorX& q, cc::VectorX& v)
  {
    const auto& fb = state.floatingBase();
    const auto& joints = state.joints();
    if(base_type == FLOATING)                                                   
    {
      q.head(7) = fb.pos();                                               
      q.tail(q.size() - 7) = joints.pos();
      v.head(6) = fb.vel(); 
      v.tail(v.size() - 6) = joints.vel();
    }
    else if(base_type == MOBILE)
    {
      cc::Scalar theta = Eigen::AngleAxisd(fb.pos().angular()) .angle();

      q.head(2) << fb.pos().linear().x(), fb.pos().linear().y();     
      q.segment(2, 2) = cc::polar_to_cart(theta);
      q.tail(q.size() - 4) = joints.pos();

      v.head(3) << fb.vel().linear().x(), fb.vel().linear().y(), fb.vel().angular().z();
      v.tail(v.size() - 3) = joints.vel();
    }
    else
    {
      q = joints.pos();
      v = joints.vel();
    }
  }
  
  void to_robot_state(const cc::VectorX& q, const cc::VectorX& v, BaseType base_type, cc::RobotState& state)
  {
    // save the solution of floatingbase state and jointstate
    auto& fb = state.floatingBase();
    auto& joints = state.joints();
    if(base_type == FLOATING)                                                   
    {
      fb.pos() = q.head(7);
      fb.vel() = v.head(6);

      joints.pos() = q.tail(q.size() - 7);
      joints.vel() = v.tail(v.size() - 6);
    }
    else if(base_type == MOBILE)
    {
      cc::Scalar theta = cc::cart_to_polar(q[3], q[2]);

      fb.pos().linear() << q[0], q[1], 0.0;
      fb.pos().angular() =  Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
      fb.vel() << q[0], q[1], 0, 0, 0, q[2];
    
      joints.pos() << q[0], q[1], theta, q.tail(12);
      joints.vel() = v;
    }
    else
    {
      joints.pos() = q;
      joints.vel() = v;
    }
  }

}