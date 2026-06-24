// Copyright 2026 ros2-arm-teleoperation-suite contributors
// SPDX-License-Identifier: MIT
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// cartesian_impedance_controller.hpp
// -----------------------------------
// ros2_control plugin implementing the Cartesian impedance law:
//
//   τ = Jᵀ [ K (x_d − x) + D (ẋ_d − ẋ) ] + g(q)
//
// Contact-adaptive stiffness: when ||f_contact|| > threshold, the
// translational stiffness K(0:3, 0:3) is scaled down proportionally,
// allowing the end-effector to compliantly yield to external forces.
//
// Implementation uses Eigen with analytic Panda FK/Jacobian (see
// impedance_math.hpp). KDL is intentionally NOT used; see §5 of
// SPEC_V2_M3_IMPEDANCE_CTRL.md for rationale.

#ifndef TELEOP_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_
#define TELEOP_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace teleop_controllers
{

/// Cartesian impedance controller (ros2_control plugin, M3).
///
/// update() runs in the controller_manager's RT thread at 1 kHz.
/// All sensor callbacks write to realtime_tools::RealtimeBuffer so
/// the RT thread never blocks on a mutex.
class CartesianImpedanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── joint layout ──────────────────────────────────────────────────────────
  std::vector<std::string> joint_names_;
  size_t num_joints_{0};

  // ── Cartesian impedance parameters ────────────────────────────────────────
  /// Diagonal stiffness [Tx Ty Tz Rx Ry Rz] (N/m and N·m/rad).
  Eigen::Matrix<double, 6, 6> K_cart_{Eigen::Matrix<double, 6, 6>::Zero()};
  /// Diagonal damping (critical: D = 2√K per axis).
  Eigen::Matrix<double, 6, 6> D_cart_{Eigen::Matrix<double, 6, 6>::Zero()};
  /// Per-joint torque limits [N·m].
  std::vector<double> max_torque_;
  /// Max Cartesian position error before clamping [m].
  double max_cart_error_m_{0.1};
  /// Joint-space posture stabilizer, used to keep the redundant arm near q_des.
  std::vector<double> joint_stiffness_;
  std::vector<double> joint_damping_;

  // ── Contact-adaptive stiffness ────────────────────────────────────────────
  double contact_threshold_n_{5.0};   ///< Force threshold [N]
  double stiffness_scale_{50.0};      ///< Decay denominator

  // ── RT-safe shared state ──────────────────────────────────────────────────
  /// Target joint positions from /joint_target (latest point, RT-safe).
  realtime_tools::RealtimeBuffer<std::vector<double>> target_positions_;
  /// Until the first explicit target arrives, hold the measured pose.
  std::atomic<bool> target_received_{false};
  /// Latest contact wrench from /ft_sensor.
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::WrenchStamped> ft_buffer_;
  /// E-Stop flag: set true by /safety/estop → τ immediately zeroed.
  std::atomic<bool> estop_active_{false};

  // ── Subscriptions ─────────────────────────────────────────────────────────
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_target_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_ft_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;

  // ── Private helpers ───────────────────────────────────────────────────────
  /// Zero all command interfaces (used on deactivate / E-Stop).
  void set_zero_torque();

  /// State interfaces are requested as position/velocity, but some
  /// controller_manager versions preserve the hardware-exported effort slot.
  size_t state_position_index(size_t joint_index) const;
  size_t state_velocity_index(size_t joint_index) const;

  /// Apply contact-adaptive stiffness: scales K_cart_ translational block
  /// based on the contact force magnitude.
  Eigen::Matrix<double, 6, 6> adapted_stiffness(
    const geometry_msgs::msg::WrenchStamped & ft) const;
};

}  // namespace teleop_controllers

#endif  // TELEOP_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_
