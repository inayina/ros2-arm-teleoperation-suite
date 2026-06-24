// Copyright 2026 ros2-arm-teleoperation-suite contributors
// SPDX-License-Identifier: MIT
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// impedance_math.hpp
// ------------------
// Franka Panda analytic forward kinematics, geometric Jacobian, and
// Cartesian error computation using Eigen.
//
// Design note: KDL is intentionally NOT used here. The Panda's DH parameters
// are fixed and publicly documented, enabling a closed-form analytic
// implementation that avoids the ros-jazzy-orocos-kdl dependency,
// is more performant at 1 kHz, and is trivially unit-testable.
//
// Reference DH parameters (modified DH, a-d-alpha convention):
//   Franka Emika Panda — Technical Specification, Table 4 (Rev. 3.0)
//
#ifndef TELEOP_CONTROLLERS__IMPEDANCE_MATH_HPP_
#define TELEOP_CONTROLLERS__IMPEDANCE_MATH_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace teleop_controllers
{
namespace impedance_math
{

/// Number of joints (Franka Panda is a 7-DOF arm).
static constexpr int kNumJoints = 7;

/// Franka Panda modified DH parameters.
/// Layout per row: [a (m), d (m), alpha (rad), offset (rad)]
/// Source: Franka Emika Panda – Technical Specification Table 4.
extern const double kDH[kNumJoints][4];

// ---------------------------------------------------------------------------
// Forward Kinematics
// ---------------------------------------------------------------------------

/// Compute the end-effector pose in the base frame.
///
/// \param q  Joint angles [rad], length kNumJoints.
/// \returns  Homogeneous transform T_{base→ee} as Eigen::Isometry3d.
Eigen::Isometry3d forward_kinematics(const Eigen::Matrix<double, 7, 1> & q);

// ---------------------------------------------------------------------------
// Jacobian
// ---------------------------------------------------------------------------

/// Compute the 6×7 geometric Jacobian in the base frame.
///
/// Columns are [Jv; Jw] for each joint (linear velocity on top,
/// angular velocity on bottom).
///
/// \param q  Joint angles [rad], length kNumJoints.
/// \returns  J ∈ ℝ^{6×7}.
Eigen::Matrix<double, 6, 7> jacobian(const Eigen::Matrix<double, 7, 1> & q);

// ---------------------------------------------------------------------------
// Cartesian error
// ---------------------------------------------------------------------------

/// Compute the 6-D Cartesian error vector [Δp; Δφ].
///
/// Position error:  Δp = p_desired − p_current (in base frame).
/// Orientation error: axis-angle Δφ derived from q_err = q_current^{-1} ⊗ q_desired,
///   expressed in the base frame (avoids gimbal lock).
///
/// \param T_current   Current end-effector pose.
/// \param T_desired   Desired end-effector pose.
/// \returns  6-vector [Δpx, Δpy, Δpz, Δφx, Δφy, Δφz].
Eigen::Matrix<double, 6, 1> cartesian_error(
  const Eigen::Isometry3d & T_current,
  const Eigen::Isometry3d & T_desired);

}  // namespace impedance_math
}  // namespace teleop_controllers

#endif  // TELEOP_CONTROLLERS__IMPEDANCE_MATH_HPP_
