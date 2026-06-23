#include "teleop_controllers/cartesian_impedance_controller.hpp"

#include <algorithm>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace teleop_controllers
{

controller_interface::CallbackReturn CartesianImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("kp", {});
    auto_declare<std::vector<double>>("kd", {});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  num_joints_ = joint_names_.size();

  kp_ = get_node()->get_parameter("kp").as_double_array();
  kd_ = get_node()->get_parameter("kd").as_double_array();
  if (kp_.size() != num_joints_) {kp_.assign(num_joints_, 40.0);}
  if (kd_.size() != num_joints_) {kd_.assign(num_joints_, 4.0);}

  target_positions_.writeFromNonRT(std::vector<double>(num_joints_, 0.0));

  sub_target_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/joint_target", rclcpp::SystemDefaultsQoS(),
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      if (!msg->points.empty()) {
        const auto & pts = msg->points.back().positions;
        if (pts.size() == num_joints_) {
          target_positions_.writeFromNonRT(
            std::vector<double>(pts.begin(), pts.end()));
        }
      }
    });

  sub_ft_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/ft_sensor", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
      ft_buffer_.writeFromNonRT(*msg);
    });

  RCLCPP_INFO(get_node()->get_logger(), "Configured with %zu joints.", num_joints_);
  return controller_interface::CallbackReturn::SUCCESS;
}

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

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Seed the target with the current measured position to avoid a jump.
  std::vector<double> current(num_joints_, 0.0);
  for (size_t i = 0; i < num_joints_; ++i) {
    current[i] = state_interfaces_[2 * i].get_optional().value_or(0.0);  // position
  }
  target_positions_.writeFromNonRT(current);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < num_joints_; ++i) {
    (void)command_interfaces_[i].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto target = *target_positions_.readFromRT();

  // TODO(M3): replace joint-space PD with Cartesian impedance:
  //   tau = J^T [K (x_d - x) + D (xdot_d - xdot)] + g(q),
  //   contact-adaptive K from ft_buffer_.
  for (size_t i = 0; i < num_joints_; ++i) {
    const double q = state_interfaces_[2 * i].get_optional().value_or(0.0);
    const double qd = state_interfaces_[2 * i + 1].get_optional().value_or(0.0);
    const double q_des = (i < target.size()) ? target[i] : q;
    const double tau = kp_[i] * (q_des - q) - kd_[i] * qd;
    (void)command_interfaces_[i].set_value(tau);
  }
  return controller_interface::return_type::OK;
}

}  // namespace teleop_controllers

PLUGINLIB_EXPORT_CLASS(
  teleop_controllers::CartesianImpedanceController, controller_interface::ControllerInterface)
