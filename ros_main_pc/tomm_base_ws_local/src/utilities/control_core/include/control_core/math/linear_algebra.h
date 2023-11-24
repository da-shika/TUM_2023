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

#ifndef CONTROL_CORE_MATH_LINEAR_ALGEBRA_H
#define CONTROL_CORE_MATH_LINEAR_ALGEBRA_H

#include <control_core/types.h>

// the namespace for the project
namespace cc
{

  /*!
  * \brief Maps an elements from cartesian vector space R^3 into an element of
  * the lie algebra of of SO(3).
  */
  template <typename Derived>
  inline Eigen::Matrix<typename Derived::Scalar,3,3>
    hat(const Eigen::MatrixBase<Derived> &v)
  {
    Matrix3 ret;
    ret << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
    return ret;
  }

  /**
   * @brief create a block diagonal matrix from a matrix A
   * 
   * bdm = np.diag(a,a,...)
   * 
   * @tparam Derived 
   * @param a 
   * @param count 
   * @return Eigen::MatrixXd 
   */
  template <typename Derived>
  inline Eigen::MatrixXd blkdiag(const Eigen::MatrixBase<Derived>& A, int count)
  {
    Eigen::MatrixXd bdm = Eigen::MatrixXd::Zero(A.rows()*count, A.cols()*count);
    for (int i = 0; i < count; ++i)
    {
      bdm.block(i * A.rows(), i * A.cols(), A.rows(), A.cols()) = A;
    }
    return bdm;
  }

  /**
   * @brief create block diagonal matrix from two matrices A and B
   * 
   * @tparam DerivedA 
   * @tparam DerivedB 
   * @param A 
   * @param B 
   * @return Eigen::MatrixXd 
   */
  template <typename DerivedA, typename DerivedB>
  inline Eigen::MatrixXd blkdiag(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedB>& B)
  {
    Eigen::MatrixXd bdm = Eigen::MatrixXd::Zero(A.rows()+B.rows(), A.cols()+B.cols());
    bdm.topLeftCorner(A.rows(), A.cols()) = A;
    bdm.bottomRightCorner(B.rows(), B.cols()) = B;
    return bdm;
  }

  inline Eigen::MatrixXd blkdiag(const std::vector<Eigen::MatrixXd>& vec)
  {
    Eigen::MatrixXd::Index rows = 0; Eigen::MatrixXd::Index cols = 0;
    for(const Eigen::MatrixXd& A : vec)
    {
      rows += A.rows();
      cols += A.cols();
    }
    Eigen::MatrixXd bdm = Eigen::MatrixXd::Zero(rows, cols);

    int i = 0; int j = 0;
    for(const Eigen::MatrixXd& A : vec)
    {
      bdm.block(i, j, A.rows(), A.cols()) = A;
      i += A.rows(); j += A.cols();
    }
    return bdm;
  }

  /**
   * @brief create a toeplitz matrix from vector v
   * 
   * m = np.hstack([
   *  cyclicShift(v,0), cyclicShift(v,1), cyclicShift(v,2), ...])
   * 
   * @tparam Derived 
   * @param a 
   * @param count 
   * @return Eigen::MatrixXd 
   */
  template <typename Derived>
  inline Eigen::MatrixXd toeplitz(const Eigen::MatrixBase<Derived>& v)
  {
    int n = v.rows();
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(n,n);

    m.diagonal(0).setConstant(v[0]);
    for (int i = 1; i < n; ++i)
    {
      m.diagonal(i).setConstant(v[i]);
      m.diagonal(-i).setConstant(v[i]);
    }
    return m;
  }

  /**
   * @brief creates a lower triangular matrix of size n
   * 
   * @param n 
   * @return Eigen::MatrixXd 
   */
  inline Eigen::MatrixXd tri(int n)
  {

    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(n,n);
    m.triangularView<Eigen::Lower>().setOnes();
    return m;
  }

} // namespace cc

#endif