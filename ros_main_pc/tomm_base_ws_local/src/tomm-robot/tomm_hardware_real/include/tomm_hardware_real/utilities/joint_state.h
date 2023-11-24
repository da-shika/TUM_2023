#ifndef TOMM_HARDWARE_JOINTSTATE_H_
#define TOMM_HARDWARE_JOINTSTATE_H_

#include <Eigen/Dense>

#define DOF_ARM 6
#define DOF_BASE 3
#define NUM_WHEELS 4

#define DEG2RAD(DEG) ((DEG) * ((M_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * ((180.0) / (M_PI)))

namespace tomm_hw
{
  // ------------ Type Definitions ---------------------------------------------
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, DOF_ARM, 1> VectorDOFArm;
  typedef Eigen::Matrix<Scalar, DOF_ARM, DOF_ARM> MatrixDOFArm;

  typedef Eigen::Matrix<Scalar, DOF_BASE, 1> VectorDOFBase;
  typedef Eigen::Matrix<Scalar, DOF_BASE, DOF_BASE> MatrixDOFBase;

  typedef Eigen::Matrix<Scalar, NUM_WHEELS, 1> VectorWheels;
  typedef Eigen::Matrix<Scalar, NUM_WHEELS, NUM_WHEELS> MatrixWheels;

  typedef Eigen::Matrix<double, 3, 1> Vector3;
  typedef Eigen::Matrix<double, 4, 1> Vector4;
  typedef Eigen::Matrix<double, 6, 1> Vector6;
  // ---------------------------------------------------------------------------

  // ------------ Joint State --------------------------------------------------
  template <int _Rows = Eigen::Dynamic>
  class JointState
  {
    typedef Eigen::Matrix<Scalar, _Rows, 1> Vector;

  private:
    Vector q_;
    Vector qP_;
    Vector qPP_;
    Vector tau_;

  public:
    JointState() : q_(Vector::Zero()),
                   qP_(Vector::Zero()),
                   qPP_(Vector::Zero()),
                   tau_(Vector::Zero())
    {
    }

    JointState(const JointState &s) : q_(s.q_),
                                      qP_(s.qP_),
                                      qPP_(s.qPP_),
                                      tau_(s.tau_)
    {
    }

    ~JointState() {}

    void setZero()
    {
      q_.setZero();
      qP_.setZero();
      qPP_.setZero();
      tau_.setZero();
    }

    Vector &q()
    {
      return q_;
    }

    Vector &pos()
    {
      return q_;
    }

    const Vector &q() const
    {
      return q_;
    }

    const Vector &pos() const
    {
      return q_;
    }

    Vector &qP()
    {
      return qP_;
    }

    Vector &vel()
    {
      return qP_;
    }

    const Vector &qP() const
    {
      return qP_;
    }

    const Vector &vel() const
    {
      return qP_;
    }

    Vector &qPP()
    {
      return qPP_;
    }

    Vector &acc()
    {
      return qPP_;
    }

    const Vector &qPP() const
    {
      return qPP_;
    }

    const Vector &acc() const
    {
      return qPP_;
    }

    Vector &tau()
    {
      return tau_;
    }

    const Vector &tau() const
    {
      return tau_;
    }
  };

}

#endif
