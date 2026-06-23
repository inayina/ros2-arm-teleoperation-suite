#include "canopen_hw_interface/canopen_system.hpp"

#include <algorithm>
#include <cctype>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace canopen_hw_interface
{

hardware_interface::CallbackReturn CanopenSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read hardware parameters from the <hardware> block.
  auto get_param = [&](const std::string & key, const std::string & def) {
    auto it = info_.hardware_parameters.find(key);
    return it != info_.hardware_parameters.end() ? it->second : def;
  };
  auto use_sim_text = get_param("use_sim", "true");
  std::transform(use_sim_text.begin(), use_sim_text.end(), use_sim_text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  use_sim_ = (use_sim_text == "true" || use_sim_text == "1" || use_sim_text == "yes");
  can_interface_ = get_param("can_interface", "vcan0");

  num_joints_ = info_.joints.size();
  hw_cmd_effort_.assign(num_joints_, 0.0);
  hw_state_position_.assign(num_joints_, 0.0);
  hw_state_velocity_.assign(num_joints_, 0.0);
  hw_state_effort_.assign(num_joints_, 0.0);
  encoder_position_.assign(num_joints_, 0.0);
  encoder_velocity_.assign(num_joints_, 0.0);
  encoder_effort_.assign(num_joints_, 0.0);

  RCLCPP_INFO(
    get_logger(), "CanopenSystem init: %zu joints, use_sim=%s, can_interface=%s",
    num_joints_, use_sim_ ? "true" : "false", can_interface_.c_str());

  // Validate interface contract: effort command + position/velocity state.
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' must expose exactly one 'effort' command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CanopenSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < num_joints_; ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_position_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_state_effort_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CanopenSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_cmd_effort_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn CanopenSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (use_sim_) {
    // Spin a private node for the /sim backplane.
    node_ = std::make_shared<rclcpp::Node>("canopen_sim_backplane");
    pub_sim_effort_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/sim/joint_effort_cmd", rclcpp::SensorDataQoS());
    sub_sim_encoder_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/sim/encoder_state", rclcpp::SensorDataQoS(),
      std::bind(&CanopenSystem::on_encoder_state, this, std::placeholders::_1));
    sub_estop_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/safety/estop", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) { estop_active_.store(msg->data); });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    running_.store(true);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    RCLCPP_INFO(get_logger(), "CanopenSystem activated in SIM mode (/sim backplane).");
  } else {
    // TODO(M2): open SocketCAN, NMT->Operational, DS402 to Operation Enabled.
    RCLCPP_WARN(
      get_logger(), "CanopenSystem activated in CAN mode on '%s' (not yet implemented).",
      can_interface_.c_str());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanopenSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (running_.exchange(false)) {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void CanopenSystem::on_encoder_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(encoder_mutex_);
  const size_t n = std::min(num_joints_, msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    encoder_position_[i] = msg->position[i];
    if (i < msg->velocity.size()) {encoder_velocity_[i] = msg->velocity[i];}
    if (i < msg->effort.size()) {encoder_effort_[i] = msg->effort[i];}
  }
  encoder_received_ = true;
}

hardware_interface::return_type CanopenSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // SIM: copy latest encoder feedback. CAN: decode TPDO (TODO M2).
  std::lock_guard<std::mutex> lock(encoder_mutex_);
  if (encoder_received_) {
    hw_state_position_ = encoder_position_;
    hw_state_velocity_ = encoder_velocity_;
    hw_state_effort_ = encoder_effort_;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanopenSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // E-stop -> command zero torque (mimics DS402 Quick Stop). TODO(M5): real QS.
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data.resize(num_joints_);
  const bool estop = estop_active_.load();
  for (size_t i = 0; i < num_joints_; ++i) {
    cmd.data[i] = estop ? 0.0 : hw_cmd_effort_[i];
  }

  if (use_sim_ && pub_sim_effort_) {
    pub_sim_effort_->publish(cmd);
  }
  // TODO(M2): encode cmd as RPDO target torque and send on the CAN bus.
  return hardware_interface::return_type::OK;
}

}  // namespace canopen_hw_interface

PLUGINLIB_EXPORT_CLASS(
  canopen_hw_interface::CanopenSystem, hardware_interface::SystemInterface)
