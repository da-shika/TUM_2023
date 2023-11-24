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


#ifndef CONTROL_CORE_PRIMITIVE_TYPES_H
#define CONTROL_CORE_PRIMITIVE_TYPES_H

////////////////////////////////////////////////////////////////////////////////
// configuration definitions
////////////////////////////////////////////////////////////////////////////////
#include <control_core/configuration.h>

////////////////////////////////////////////////////////////////////////////////
// basic types
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types/matrix.h>
#include <control_core/types/rotation3.h>
#include <control_core/types/vector3.h>
#include <control_core/types/vector_dof.h>

////////////////////////////////////////////////////////////////////////////////
// state types
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types/joint_state.h>
#include <control_core/types/angular_state.h>
#include <control_core/types/linear_state.h>
#include <control_core/types/cartesian_state.h>

////////////////////////////////////////////////////////////////////////////////
// effort types
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types/joint_effort.h>
#include <control_core/types/moment.h>
#include <control_core/types/force.h>
#include <control_core/types/wrench.h>

////////////////////////////////////////////////////////////////////////////////
// cartesian types
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types/spatial_vector.h>
#include <control_core/types/cartesian_vector.h>
#include <control_core/types/homogeneous_transformation.h>

////////////////////////////////////////////////////////////////////////////////
// control types
////////////////////////////////////////////////////////////////////////////////
#include <control_core/types/jacobian.h>

////////////////////////////////////////////////////////////////////////////////
// eigen types
////////////////////////////////////////////////////////////////////////////////
#include <control_core/utilities/eigen_typedef_macros.h>

/*!
 * \brief The namespace for the configured types.
 */
namespace cc
{
  /*! Type for Scalars */
  typedef CC_TYPES_SCALAR Scalar;

  /*! Basic Types */
  typedef control_core::Rotation3<Scalar> Rotation3;
  typedef control_core::Vector3<Scalar> Vector3;

  typedef control_core::AngularPosition<Scalar> AngularPosition;
  typedef control_core::AngularVelocity<Scalar> AngularVelocity;
  typedef control_core::AngularAcceleration<Scalar> AngularAcceleration;
  typedef control_core::Moment<Scalar> Moment;
  typedef control_core::AngularState<Scalar> AngularState;

  typedef control_core::LinearPosition<Scalar> LinearPosition;
  typedef control_core::LinearVelocity<Scalar> LinearVelocity;
  typedef control_core::LinearAcceleration<Scalar> LinearAcceleration;
  typedef control_core::Force<Scalar> Force;
  typedef control_core::LinearState<Scalar> LinearState;

  typedef control_core::HomogeneousTransformation<Scalar> HomogeneousTransformation;

  typedef control_core::CartesianPosition<Scalar> CartesianPosition;
  typedef control_core::CartesianVelocity<Scalar> CartesianVelocity;
  typedef control_core::CartesianAcceleration<Scalar> CartesianAcceleration;
  typedef control_core::Wrench<Scalar> Wrench;
  typedef control_core::CartesianState<Scalar> CartesianState;

  typedef control_core::SpatialVector<Scalar> SpatialVector;
  typedef control_core::CartesianVector<Scalar> CartesianVector;

  typedef control_core::JointPosition<Scalar> JointPosition;
  typedef control_core::JointVelocity<Scalar> JointVelocity;
  typedef control_core::JointAcceleration<Scalar> JointAcceleration;
  typedef control_core::JointEffort<Scalar> JointEffort;
  typedef control_core::JointState<Scalar> JointState;

  /*! Eigen specific types */
  CONTROL_CORE_TYPEDEFS_ALL_SIZES(unsigned int, ui)
  CONTROL_CORE_TYPEDEFS_ALL_SIZES(int, i)

  typedef control_core::Matrix<Scalar, 6, 1> Vector6;
  typedef control_core::Matrix<Scalar, 7, 1> Vector7;
  typedef control_core::Matrix<Scalar, 4, 1> Vector4;
  typedef control_core::Matrix<Scalar, 2, 1> Vector2;

  typedef control_core::Matrix<Scalar, 2, 2> Matrix2;
  typedef control_core::Matrix<Scalar, 3, 3> Matrix3;
  typedef control_core::Matrix<Scalar, 4, 4> Matrix4;
  typedef control_core::Matrix<Scalar, 6, 6> Matrix6;

  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

  template<int ROWS>
  using Vector = Eigen::Matrix<Scalar, ROWS, 1>;

  template<int ROWS, int COLS>
  using Matrix = Eigen::Matrix<Scalar, ROWS, COLS>;

  typedef Vector2 Point2d;
  typedef Vector3 Point3d;
  typedef Eigen::Matrix<Scalar, 2, Eigen::Dynamic> Points2d;
  typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Points3d;
}

#endif  // CONTROL_CORE_PRIMITIVE_TYPES_H
