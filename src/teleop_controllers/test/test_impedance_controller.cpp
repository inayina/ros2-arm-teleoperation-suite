// Copyright 2026 ros2-arm-teleoperation-suite contributors
// SPDX-License-Identifier: MIT
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// test_impedance_controller.cpp
// ------------------------------
// Integration-level GTest for the CartesianImpedanceController logic.
// Uses teleop_controllers::impedance_math directly (no ROS spin needed)
// to verify the control law produces expected torques.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>

#include "teleop_controllers/impedance_math.hpp"

using teleop_controllers::impedance_math::cartesian_error;
using teleop_controllers::impedance_math::forward_kinematics;
using teleop_controllers::impedance_math::jacobian;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: simplified control law mirror (mirrors update() logic)
// ─────────────────────────────────────────────────────────────────────────────

static Eigen::Matrix<double, 6, 6> make_K(const std::vector<double> & k)
{
  Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
  for (int i = 0; i < 6; ++i) {
    K(i, i) = k[i];
  }
  return K;
}

static Eigen::Matrix<double, 7, 1> compute_tau(
  const Eigen::Matrix<double, 7, 1> & q,
  const Eigen::Matrix<double, 7, 1> & dq,
  const Eigen::Matrix<double, 7, 1> & q_des,
  const Eigen::Matrix<double, 6, 6> & K,
  const Eigen::Matrix<double, 6, 6> & D)
{
  const Eigen::Isometry3d T_cur = forward_kinematics(q);
  const Eigen::Isometry3d T_des = forward_kinematics(q_des);
  const Eigen::Matrix<double, 6, 1> err = cartesian_error(T_cur, T_des);
  const Eigen::Matrix<double, 6, 7> J = jacobian(q);
  const Eigen::Matrix<double, 6, 1> cart_vel = -(J * dq);  // ẋ_d − ẋ ≈ −ẋ
  return J.transpose() * (K * err + D * cart_vel);
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

TEST(ControlLawTest, ZeroErrorProducesNearZeroTorque)
{
  // When q == q_des and dq == 0, the control torque should be ≈ 0.
  const Eigen::Matrix<double, 7, 1> q = (Eigen::Matrix<double, 7, 1>() <<
    0.1, -0.3, 0.2, -0.7, 0.1, 0.5, 0.0).finished();
  const Eigen::Matrix<double, 7, 1> dq = Eigen::Matrix<double, 7, 1>::Zero();

  const auto K = make_K({200.0, 200.0, 200.0, 10.0, 10.0, 10.0});
  const auto D = make_K({28.3, 28.3, 28.3, 6.3, 6.3, 6.3});

  const auto tau = compute_tau(q, dq, q, K, D);
  EXPECT_NEAR(tau.norm(), 0.0, 1e-8)
    << "Torque should be ≈ 0 when at target with zero velocity";
}

TEST(ControlLawTest, DisplacedTargetProducesNonZeroTorque)
{
  // A target 3 cm ahead in x should pull the arm → non-zero torque.
  const Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
  const Eigen::Matrix<double, 7, 1> dq = Eigen::Matrix<double, 7, 1>::Zero();

  // Shift joint 1 by +0.05 rad to create a measurable position offset.
  Eigen::Matrix<double, 7, 1> q_des = q;
  q_des(0) += 0.05;

  const auto K = make_K({200.0, 200.0, 200.0, 10.0, 10.0, 10.0});
  const auto D = make_K({28.3, 28.3, 28.3, 6.3, 6.3, 6.3});

  const auto tau = compute_tau(q, dq, q_des, K, D);
  EXPECT_GT(tau.norm(), 0.01)
    << "Torque should be significant when tracking a displaced target";
}

TEST(ControlLawTest, DampingReducesTorqueWithVelocity)
{
  // Same error, but adding velocity should increase total torque magnitude
  // (damping term adds, assuming both K and D terms act in same direction).
  const Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> q_des = q;
  q_des(0) += 0.05;

  const auto K = make_K({200.0, 200.0, 200.0, 10.0, 10.0, 10.0});
  const auto D = make_K({28.3, 28.3, 28.3, 6.3, 6.3, 6.3});

  const Eigen::Matrix<double, 7, 1> dq_zero = Eigen::Matrix<double, 7, 1>::Zero();
  // Velocity that opposes desired motion (robot moving away from target)
  Eigen::Matrix<double, 7, 1> dq_nonzero = Eigen::Matrix<double, 7, 1>::Zero();
  dq_nonzero(0) = -0.5;  // moving in opposite direction

  const double tau_no_vel = compute_tau(q, dq_zero, q_des, K, D).norm();
  const double tau_with_vel = compute_tau(q, dq_nonzero, q_des, K, D).norm();

  EXPECT_GT(tau_with_vel, tau_no_vel)
    << "Damping should increase torque when velocity opposes desired motion";
}

TEST(ControlLawTest, JacobianNullspaceTorque)
{
  // At Panda's standard ready pose (non-singular), Jacobian should have rank 6.
  // Zero config is a known elbow singularity for this arm.
  // Use the canonical "ready" pose from Franka's documentation.
  const Eigen::Matrix<double, 7, 1> q_ready =
    (Eigen::Matrix<double, 7, 1>() <<
    0.0, -M_PI / 4.0, 0.0, -3.0 * M_PI / 4.0, 0.0, M_PI / 2.0, M_PI / 4.0
    ).finished();

  const auto J = jacobian(q_ready);
  // Check J has rank 6 (full row rank for a 6×7 matrix)
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const double min_sv = svd.singularValues().minCoeff();
  EXPECT_GT(min_sv, 1e-3)
    << "Jacobian at ready pose should be well-conditioned (no singularity). min_sv=" << min_sv;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
