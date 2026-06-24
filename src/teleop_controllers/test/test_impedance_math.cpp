// Copyright 2026 ros2-arm-teleoperation-suite contributors
// SPDX-License-Identifier: MIT
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// test_impedance_math.cpp
// -----------------------
// GTest suite for teleop_controllers::impedance_math:
//   - FK zero-pose (all joints = 0): checks known translation.
//   - FK at a known non-trivial configuration vs reference.
//   - Jacobian numerical-derivative consistency.
//   - cartesian_error sign and magnitude.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <cmath>

#include "teleop_controllers/impedance_math.hpp"

using teleop_controllers::impedance_math::cartesian_error;
using teleop_controllers::impedance_math::forward_kinematics;
using teleop_controllers::impedance_math::jacobian;
using teleop_controllers::impedance_math::kNumJoints;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static Eigen::Matrix<double, 7, 1> zero_q()
{
  return Eigen::Matrix<double, 7, 1>::Zero();
}

// ─────────────────────────────────────────────────────────────────────────────
// Forward kinematics tests
// ─────────────────────────────────────────────────────────────────────────────

TEST(FKTest, ZeroConfigTranslationZ)
{
  // At q = 0 the arm hangs down; the EE translation should be along +z
  // The total reach along z equals d1 + d3 + d5 = 0.333 + 0.316 + 0.384 = 1.033 m.
  const auto T = forward_kinematics(zero_q());
  EXPECT_NEAR(T.translation().z(), 1.033, 1e-3)
    << "EE z-translation at zero config should ≈ 1.033 m";
  // At zero config x should be a7 + a4 - a5 = 0.088 + 0.0825 - 0.0825 = 0.088 m
  EXPECT_NEAR(T.translation().x(), 0.088, 1e-3)
    << "EE x-translation at zero config should ≈ 0.088 m";
}

TEST(FKTest, RotationIsOrthonormal)
{
  // The rotation part must be SO(3): R^T R ≈ I, det ≈ 1
  const Eigen::Matrix<double, 7, 1> q = (Eigen::Matrix<double, 7, 1>() <<
    0.1, -0.2, 0.3, -0.5, 0.4, 0.6, -0.3).finished();
  const auto T = forward_kinematics(q);
  const Eigen::Matrix3d RtR = T.rotation().transpose() * T.rotation();
  EXPECT_NEAR((RtR - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-10)
    << "Rotation must be orthonormal";
  EXPECT_NEAR(T.rotation().determinant(), 1.0, 1e-10)
    << "Rotation determinant must be 1";
}

// ─────────────────────────────────────────────────────────────────────────────
// Jacobian tests
// ─────────────────────────────────────────────────────────────────────────────

TEST(JacobianTest, NumericalDerivativeConsistency)
{
  // Verify J * dq ≈ finite-difference of FK translation.
  const Eigen::Matrix<double, 7, 1> q = (Eigen::Matrix<double, 7, 1>() <<
    0.2, -0.1, 0.4, -0.8, 0.3, 0.5, -0.2).finished();

  const Eigen::Matrix<double, 6, 7> J = jacobian(q);
  const double eps = 1e-6;

  for (int j = 0; j < kNumJoints; ++j) {
    Eigen::Matrix<double, 7, 1> q_plus = q;
    q_plus(j) += eps;
    const Eigen::Vector3d dp =
      (forward_kinematics(q_plus).translation() - forward_kinematics(q).translation()) / eps;
    // Compare linear Jacobian column with numerical derivative
    EXPECT_NEAR((J.block<3, 1>(0, j) - dp).norm(), 0.0, 1e-4)
      << "Jacobian linear column " << j << " mismatch";
  }
}

TEST(JacobianTest, ShapeIs6x7)
{
  const auto J = jacobian(zero_q());
  EXPECT_EQ(J.rows(), 6);
  EXPECT_EQ(J.cols(), 7);
}

// ─────────────────────────────────────────────────────────────────────────────
// Cartesian error tests
// ─────────────────────────────────────────────────────────────────────────────

TEST(CartesianErrorTest, IdentityTransforms)
{
  // Zero error when current == desired
  const Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const auto err = cartesian_error(T, T);
  EXPECT_NEAR(err.norm(), 0.0, 1e-12);
}

TEST(CartesianErrorTest, PureTranslationError)
{
  Eigen::Isometry3d T_cur = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_des = Eigen::Isometry3d::Identity();
  T_des.translation() = Eigen::Vector3d(0.05, 0.0, 0.0);  // 5 cm in x

  const auto err = cartesian_error(T_cur, T_des);
  EXPECT_NEAR(err(0), 0.05, 1e-10);
  EXPECT_NEAR(err.tail<3>().norm(), 0.0, 1e-10);
}

TEST(CartesianErrorTest, PureRotationError)
{
  Eigen::Isometry3d T_cur = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_des = Eigen::Isometry3d::Identity();
  // 90° rotation about z
  T_des.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto err = cartesian_error(T_cur, T_des);
  EXPECT_NEAR(err.head<3>().norm(), 0.0, 1e-10);
  EXPECT_NEAR(err.tail<3>().norm(), M_PI / 2.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
