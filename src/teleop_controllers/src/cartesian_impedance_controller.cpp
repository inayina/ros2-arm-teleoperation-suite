// Copyright 2026 ros2-arm-teleoperation-suite contributors
// SPDX-License-Identifier: MIT
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// cartesian_impedance_controller.cpp
// ------------------------------------
// Implements the Cartesian impedance control law:
//
//   τ = Jᵀ [ K_eff (x_d − x) + D_eff (ẋ_d − ẋ) ] + g(q)
//
// where K_eff is the possibly contact-adapted stiffness and g(q) = 0
// (MuJoCo already compensates gravity at the physics layer; a full
// gravity model from KDL/URDF will be added in M4 if needed).
//
// Analytic Panda FK and Jacobian are provided by impedance_math.hpp
// (no KDL dependency; see SPEC_V2_M3_IMPEDANCE_CTRL.md §5 for rationale).

#include "teleop_controllers/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <cmath>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "teleop_controllers/impedance_math.hpp"
#include "teleop_controllers/joint_trajectory_mapping.hpp"

namespace teleop_controllers
{

// ─────────────────────────────────────────────────────────────────────────────
// on_init: declare parameters
// ─────────────────────────────────────────────────────────────────────────────
controller_interface::CallbackReturn CartesianImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    // Cartesian stiffness [Tx Ty Tz Rx Ry Rz] (N/m and N·m/rad)
    auto_declare<std::vector<double>>(
      "cartesian_stiffness", {50.0, 50.0, 50.0, 2.0, 2.0, 2.0});
    // Cartesian damping  [Tx Ty Tz Rx Ry Rz]
    auto_declare<std::vector<double>>(
      "cartesian_damping", {14.1, 14.1, 14.1, 2.8, 2.8, 2.8});
    // Per-joint max torque [N·m] (Panda limits)
    auto_declare<std::vector<double>>(
      "max_torque_nm", {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0});
    auto_declare<std::vector<double>>(
      "joint_stiffness", {35.0, 35.0, 30.0, 25.0, 12.0, 10.0, 8.0});
    auto_declare<std::vector<double>>(
      "joint_damping", {8.0, 8.0, 7.0, 6.0, 3.0, 2.5, 2.0});
    auto_declare<double>("contact_threshold_n", 5.0);
    auto_declare<double>("stiffness_scale", 50.0);
    auto_declare<double>("max_cartesian_error_m", 0.1);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure: read parameters, build K/D matrices, create subscriptions
// ─────────────────────────────────────────────────────────────────────────────
controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  num_joints_ = joint_names_.size();
  if (num_joints_ != impedance_math::kNumJoints) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %d joints for Panda, got %zu.", impedance_math::kNumJoints, num_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }

  // ── Stiffness matrix (diagonal 6×6) ────────────────────────────────────
  const auto k_vec = get_node()->get_parameter("cartesian_stiffness").as_double_array();
  const auto d_vec = get_node()->get_parameter("cartesian_damping").as_double_array();
  K_cart_.setZero();
  D_cart_.setZero();
  for (int i = 0; i < 6; ++i) {
    K_cart_(i, i) = (static_cast<int>(k_vec.size()) > i) ? k_vec[i] : 0.0;
    D_cart_(i, i) = (static_cast<int>(d_vec.size()) > i) ? d_vec[i] : 0.0;
  }

  // ── Per-joint torque limits ─────────────────────────────────────────────
  max_torque_ = get_node()->get_parameter("max_torque_nm").as_double_array();
  if (max_torque_.size() < num_joints_) {
    max_torque_.assign(num_joints_, 87.0);
  }
  joint_stiffness_ = get_node()->get_parameter("joint_stiffness").as_double_array();
  joint_damping_ = get_node()->get_parameter("joint_damping").as_double_array();
  if (joint_stiffness_.size() < num_joints_) {
    joint_stiffness_.assign(num_joints_, 20.0);
  }
  if (joint_damping_.size() < num_joints_) {
    joint_damping_.assign(num_joints_, 4.0);
  }

  // ── Contact compliance parameters ───────────────────────────────────────
  contact_threshold_n_ = get_node()->get_parameter("contact_threshold_n").as_double();
  stiffness_scale_ = get_node()->get_parameter("stiffness_scale").as_double();
  max_cart_error_m_ = get_node()->get_parameter("max_cartesian_error_m").as_double();

  // ── Seed RT buffers ─────────────────────────────────────────────────────
  target_positions_.writeFromNonRT(std::vector<double>(num_joints_, 0.0));

  // ── Subscriptions ────────────────────────────────────────────────────────
  sub_target_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/joint_target", rclcpp::SystemDefaultsQoS(),
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      std::vector<double> mapped;
      if (map_joint_trajectory_target(*msg, joint_names_, mapped)) {
        target_positions_.writeFromNonRT(mapped);
        target_received_.store(true);
      }
    });

  sub_ft_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/ft_sensor", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
      ft_buffer_.writeFromNonRT(*msg);
    });

  sub_estop_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/safety/estop", rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      estop_active_.store(msg->data);
      if (msg->data) {
        RCLCPP_WARN(get_node()->get_logger(), "E-Stop received: zeroing torque.");
      }
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "CartesianImpedanceController configured: %zu joints, "
    "K=[%.1f %.1f %.1f %.1f %.1f %.1f], D=[%.1f %.1f %.1f %.1f %.1f %.1f].",
    num_joints_,
    K_cart_(0, 0), K_cart_(1, 1), K_cart_(2, 2),
    K_cart_(3, 3), K_cart_(4, 4), K_cart_(5, 5),
    D_cart_(0, 0), D_cart_(1, 1), D_cart_(2, 2),
    D_cart_(3, 3), D_cart_(4, 4), D_cart_(5, 5));

  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// Interface configurations
// ─────────────────────────────────────────────────────────────────────────────
controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate: seed target with current pose to avoid torque jump
// ─────────────────────────────────────────────────────────────────────────────
controller_interface::CallbackReturn CartesianImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Do not clear estop_active_ here — /safety/estop (Transient Local) is authoritative.

  std::vector<double> current(num_joints_, 0.0);
  for (size_t i = 0; i < num_joints_; ++i) {
    current[i] = state_interfaces_[state_position_index(i)].get_optional().value_or(0.0);
  }
  target_positions_.writeFromNonRT(current);

  RCLCPP_INFO(get_node()->get_logger(), "CartesianImpedanceController activated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_deactivate: zero torques
// ─────────────────────────────────────────────────────────────────────────────
controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  set_zero_torque();
  return controller_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// update: 1 kHz Cartesian impedance control law
// ─────────────────────────────────────────────────────────────────────────────
controller_interface::return_type CartesianImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // ── 0. E-Stop: highest priority ─────────────────────────────────────────
  if (estop_active_.load()) {
    set_zero_torque();
    return controller_interface::return_type::OK;
  }

  // ── 1. Read current joint state ─────────────────────────────────────────
  Eigen::Matrix<double, 7, 1> q, dq;
  for (size_t i = 0; i < num_joints_; ++i) {
    q(i) = state_interfaces_[state_position_index(i)].get_optional().value_or(0.0);
    dq(i) = state_interfaces_[state_velocity_index(i)].get_optional().value_or(0.0);
  }

  // ── 2. Forward kinematics: q → T_current ────────────────────────────────
  const Eigen::Isometry3d T_current = impedance_math::forward_kinematics(q);

  // ── 3. Desired pose from /joint_target → FK ─────────────────────────────
  const auto target_q_vec = *target_positions_.readFromRT();
  Eigen::Matrix<double, 7, 1> q_des;
  for (size_t i = 0; i < num_joints_; ++i) {
    q_des(i) = (target_received_.load() && i < target_q_vec.size()) ? target_q_vec[i] : q(i);
  }
  const Eigen::Isometry3d T_desired = impedance_math::forward_kinematics(q_des);

  // ── 4. Cartesian error [Δp; Δφ] ─────────────────────────────────────────
  Eigen::Matrix<double, 6, 1> cart_error =
    impedance_math::cartesian_error(T_current, T_desired);

  // Clamp position error magnitude to prevent runaway torques
  {
    const double pos_err = cart_error.head<3>().norm();
    if (pos_err > max_cart_error_m_) {
      cart_error.head<3>() *= (max_cart_error_m_ / pos_err);
    }
  }

  // ── 5. Cartesian velocity error (Jacobian × joint velocity) ─────────────
  const Eigen::Matrix<double, 6, 7> J = impedance_math::jacobian(q);
  const Eigen::Matrix<double, 6, 1> cart_vel_err = -(J * dq);  // ẋ_d − ẋ ≈ −ẋ

  // ── 6. Contact-adaptive stiffness ───────────────────────────────────────
  const auto & ft = *ft_buffer_.readFromRT();
  const Eigen::Matrix<double, 6, 6> K_eff = adapted_stiffness(ft);

  // ── 7. Impedance control law: τ = Jᵀ [K_eff Δx + D Δẋ] ─────────────────
  // g(q) = 0: MuJoCo compensates gravity at the physics layer (M3 scope).
  Eigen::Matrix<double, 7, 1> tau_cmd =
    J.transpose() * (K_eff * cart_error + D_cart_ * cart_vel_err);
  for (size_t i = 0; i < num_joints_; ++i) {
    // The joint posture term stabilizes the redundant Panda arm around the
    // commanded Servo joint target while Cartesian impedance remains the main
    // end-effector behavior.
    tau_cmd(i) += joint_stiffness_[i] * (q_des(i) - q(i)) - joint_damping_[i] * dq(i);
  }

  // ── 8. Torque limits + write command interfaces ─────────────────────────
  for (size_t i = 0; i < num_joints_; ++i) {
    const double tau = std::clamp(tau_cmd(i), -max_torque_[i], max_torque_[i]);
    (void)command_interfaces_[i].set_value(tau);
  }

  return controller_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

void CartesianImpedanceController::set_zero_torque()
{
  for (size_t i = 0; i < num_joints_; ++i) {
    (void)command_interfaces_[i].set_value(0.0);
  }
}

size_t CartesianImpedanceController::state_position_index(size_t joint_index) const
{
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    if (state_interfaces_[i].get_prefix_name() == joint_names_[joint_index] &&
      state_interfaces_[i].get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      return i;
    }
  }
  const size_t stride = state_interfaces_.size() >= num_joints_ * 3 ? 3 : 2;
  return stride * joint_index;
}

size_t CartesianImpedanceController::state_velocity_index(size_t joint_index) const
{
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    if (state_interfaces_[i].get_prefix_name() == joint_names_[joint_index] &&
      state_interfaces_[i].get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      return i;
    }
  }
  const size_t stride = state_interfaces_.size() >= num_joints_ * 3 ? 3 : 2;
  return stride * joint_index + 1;
}

Eigen::Matrix<double, 6, 6> CartesianImpedanceController::adapted_stiffness(
  const geometry_msgs::msg::WrenchStamped & ft) const
{
  Eigen::Matrix<double, 6, 6> K_eff = K_cart_;

  const double fx = ft.wrench.force.x;
  const double fy = ft.wrench.force.y;
  const double fz = ft.wrench.force.z;
  const double fn = std::sqrt(fx * fx + fy * fy + fz * fz);

  if (fn > contact_threshold_n_) {
    // Scale down translational stiffness proportionally with contact force.
    // α ∈ [0.1, 1.0] decreases as force grows.
    const double alpha = std::max(0.1, 1.0 - (fn - contact_threshold_n_) / stiffness_scale_);
    for (int i = 0; i < 3; ++i) {
      K_eff(i, i) *= alpha;
    }
  }

  return K_eff;
}

}  // namespace teleop_controllers

PLUGINLIB_EXPORT_CLASS(
  teleop_controllers::CartesianImpedanceController, controller_interface::ControllerInterface)
