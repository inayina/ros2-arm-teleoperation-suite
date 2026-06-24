// Copyright 2026 ros2-arm-teleoperation-suite contributors
// SPDX-License-Identifier: MIT
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// impedance_math.cpp
// ------------------
// Franka Panda analytic forward kinematics and geometric Jacobian.
//
// DH convention: modified DH (Craig convention).
//   Row i = [a_{i-1}, d_i, alpha_{i-1}, theta_offset_i]
//   T_{i-1,i} = Rot_x(alpha) * Trans_x(a) * Rot_z(theta+offset) * Trans_z(d)
//
// Reference: Franka Emika Panda Technical Specification, Table 4.
//            https://frankaemika.github.io/docs/control_parameters.html

#include "teleop_controllers/impedance_math.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace teleop_controllers
{
namespace impedance_math
{

// ---------------------------------------------------------------------------
// Panda modified DH parameters [a(m), d(m), alpha(rad), offset(rad)]
// ---------------------------------------------------------------------------
const double kDH[kNumJoints][4] = {
  // a        d        alpha           offset
  {0.0, 0.333, 0.0, 0.0},                               // joint 1
  {0.0, 0.0, -M_PI / 2.0, 0.0},                         // joint 2
  {0.0, 0.316, M_PI / 2.0, 0.0},                        // joint 3
  {0.0825, 0.0, M_PI / 2.0, 0.0},                       // joint 4
  {-0.0825, 0.384, -M_PI / 2.0, 0.0},                   // joint 5
  {0.0, 0.0, M_PI / 2.0, 0.0},                          // joint 6
  {0.088, 0.0, M_PI / 2.0, 0.0},                        // joint 7
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a single modified-DH link transform T_{i-1,i}.
/// T = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)
static Eigen::Isometry3d dh_link(double a, double d, double alpha, double theta)
{
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  const double ct = std::cos(theta), st = std::sin(theta);

  Eigen::Matrix4d T;
  T << ct, -st, 0.0, a,
    st * ca, ct * ca, -sa, -sa * d,
    st * sa, ct * sa, ca, ca * d,
    0.0, 0.0, 0.0, 1.0;

  return Eigen::Isometry3d(T);
}

// ---------------------------------------------------------------------------
// Forward kinematics
// ---------------------------------------------------------------------------

Eigen::Isometry3d forward_kinematics(const Eigen::Matrix<double, 7, 1> & q)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  for (int i = 0; i < kNumJoints; ++i) {
    const double a = kDH[i][0];
    const double d = kDH[i][1];
    const double alpha = kDH[i][2];
    const double theta = q(i) + kDH[i][3];
    T = T * dh_link(a, d, alpha, theta);
  }
  return T;
}

// ---------------------------------------------------------------------------
// Geometric Jacobian
// ---------------------------------------------------------------------------

Eigen::Matrix<double, 6, 7> jacobian(const Eigen::Matrix<double, 7, 1> & q)
{
  // Compute each joint's frame origin and z-axis in the base frame.
  // For geometric Jacobian:
  //   Joint i: z_i = z-axis of frame i in base (after applying DH up to joint i)
  //            p_i = origin of frame i in base (used as the point the axis passes through)
  //   Column i: [z_i × (p_ee - p_i); z_i]

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector3d p[kNumJoints + 1];  // p[i] = origin of frame i in base
  Eigen::Vector3d z[kNumJoints];      // z[i] = z-axis of frame i in base

  p[0] = Eigen::Vector3d::Zero();  // base frame origin

  for (int i = 0; i < kNumJoints; ++i) {
    const double a = kDH[i][0];
    const double d = kDH[i][1];
    const double alpha = kDH[i][2];
    const double theta = q(i) + kDH[i][3];
    T = T * dh_link(a, d, alpha, theta);
    z[i] = T.rotation().col(2);    // z-axis of frame i in base
    p[i + 1] = T.translation();    // origin of frame i in base
  }

  // p_ee = p[kNumJoints] (end-effector origin in base frame)
  const Eigen::Vector3d p_ee = p[kNumJoints];

  Eigen::Matrix<double, 6, 7> J;
  for (int i = 0; i < kNumJoints; ++i) {
    // In modified DH (Craig), joint i rotates about the z-axis of frame i.
    // Frame i's origin (p[i+1]) is the point through which the joint axis passes.
    // Linear velocity: Jv_i = z_i × (p_ee − p[i+1])
    J.block<3, 1>(0, i) = z[i].cross(p_ee - p[i + 1]);
    // Angular velocity: Jw_i = z_i
    J.block<3, 1>(3, i) = z[i];
  }

  return J;
}

// ---------------------------------------------------------------------------
// Cartesian error
// ---------------------------------------------------------------------------

Eigen::Matrix<double, 6, 1> cartesian_error(
  const Eigen::Isometry3d & T_current,
  const Eigen::Isometry3d & T_desired)
{
  Eigen::Matrix<double, 6, 1> error;

  // --- Position error (in base frame) ---
  error.head<3>() = T_desired.translation() - T_current.translation();

  // --- Orientation error via quaternion (avoids gimbal lock) ---
  // q_err = q_current^{-1} ⊗ q_desired (error in current body frame)
  Eigen::Quaterniond q_cur(T_current.rotation());
  Eigen::Quaterniond q_des(T_desired.rotation());

  // Ensure shortest-path rotation (prevent sign flip)
  if (q_cur.dot(q_des) < 0.0) {
    q_des.coeffs() = -q_des.coeffs();
  }

  const Eigen::Quaterniond q_err = q_cur.inverse() * q_des;
  // Convert to axis-angle, express in base frame
  const Eigen::AngleAxisd aa(q_err);
  error.tail<3>() = T_current.rotation() * (aa.angle() * aa.axis());

  return error;
}

}  // namespace impedance_math
}  // namespace teleop_controllers
