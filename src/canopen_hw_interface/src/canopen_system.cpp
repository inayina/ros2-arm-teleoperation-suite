#include "canopen_hw_interface/canopen_system.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstring>
#include <limits>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace canopen_hw_interface
{

namespace
{

constexpr double kTorqueScale = 0.001;
constexpr double kVelocityScale = 0.001;
constexpr int kCountsPerRev = 131072;
constexpr double kTwoPi = 2.0 * M_PI;

constexpr uint32_t kRpdoBase = 0x200;
constexpr uint32_t kTpdo1Base = 0x180;
constexpr uint32_t kTpdo2Base = 0x280;
constexpr uint32_t kSyncCobId = 0x080;
constexpr uint32_t kNmtCobId = 0x000;
constexpr uint32_t kSdoRxBase = 0x600;

int16_t clamp_i16(int32_t v)
{
  return static_cast<int16_t>(std::max<int32_t>(-32768, std::min<int32_t>(32767, v)));
}

}  // namespace

hardware_interface::CallbackReturn CanopenSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

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
  node_ids_.resize(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    uint8_t node_id = static_cast<uint8_t>(i + 1);
    auto it = info_.joints[i].parameters.find("node_id");
    if (it != info_.joints[i].parameters.end()) {
      try {
        node_id = static_cast<uint8_t>(std::stoi(it->second));
      } catch (const std::exception &) {
        RCLCPP_WARN(
          get_logger(), "Invalid node_id for joint '%s', defaulting to %zu.",
          info_.joints[i].name.c_str(), i + 1);
      }
    }
    node_ids_[i] = node_id;
  }

  hw_cmd_effort_.assign(num_joints_, 0.0);
  hw_state_position_.assign(num_joints_, 0.0);
  hw_state_velocity_.assign(num_joints_, 0.0);
  hw_state_effort_.assign(num_joints_, 0.0);
  encoder_position_.assign(num_joints_, 0.0);
  encoder_velocity_.assign(num_joints_, 0.0);
  encoder_effort_.assign(num_joints_, 0.0);
  tpdo_position_.assign(num_joints_, 0.0);
  tpdo_velocity_.assign(num_joints_, 0.0);
  tpdo_torque_.assign(num_joints_, 0.0);

  RCLCPP_INFO(
    get_logger(), "CanopenSystem init: %zu joints, use_sim=%s, can_interface=%s",
    num_joints_, use_sim_ ? "true" : "false", can_interface_.c_str());

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

bool CanopenSystem::open_can_socket()
{
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to create CAN socket for '%s'.", can_interface_.c_str());
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(get_logger(), "CAN interface '%s' not found.", can_interface_.c_str());
    close_can_socket();
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to bind CAN socket to '%s'.", can_interface_.c_str());
    close_can_socket();
    return false;
  }

  struct timeval tv {};
  tv.tv_sec = 0;
  tv.tv_usec = 1000;
  setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  return true;
}

void CanopenSystem::close_can_socket()
{
  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
  }
}

bool CanopenSystem::send_can_frame(uint32_t cob_id, const uint8_t * data, uint8_t dlc)
{
  if (can_socket_ < 0) {
    return false;
  }
  struct can_frame frame {};
  frame.can_id = cob_id;
  frame.can_dlc = dlc;
  if (data != nullptr) {
    std::memcpy(frame.data, data, dlc);
  }
  return ::write(can_socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool CanopenSystem::send_sync_frame()
{
  return send_can_frame(kSyncCobId, nullptr, 0);
}

bool CanopenSystem::send_nmt_start()
{
  const uint8_t data[2] = {0x01, 0x00};  // Start all nodes
  return send_can_frame(kNmtCobId, data, 2);
}

bool CanopenSystem::sdo_write_u16(uint8_t node_id, uint16_t index, uint16_t value)
{
  const uint8_t data[8] = {
    0x2B,
    static_cast<uint8_t>(index & 0xFF),
    static_cast<uint8_t>((index >> 8) & 0xFF),
    0x00,
    static_cast<uint8_t>(value & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF),
    0x00,
    0x00,
  };
  return send_can_frame(kSdoRxBase + node_id, data, 8);
}

void CanopenSystem::ds402_enable_all()
{
  send_nmt_start();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  for (uint8_t node_id : node_ids_) {
    sdo_write_u16(node_id, 0x6040, 0x0006);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    sdo_write_u16(node_id, 0x6040, 0x0007);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    sdo_write_u16(node_id, 0x6040, 0x000F);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

std::vector<uint8_t> CanopenSystem::encode_rpdo_torque(double torque_nm)
{
  const int16_t raw = clamp_i16(static_cast<int32_t>(std::lround(torque_nm / kTorqueScale)));
  std::vector<uint8_t> data(8, 0);
  data[0] = static_cast<uint8_t>(raw & 0xFF);
  data[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  return data;
}

void CanopenSystem::decode_tpdo1(size_t joint_idx, const uint8_t * data)
{
  int32_t pos_cnt = static_cast<int32_t>(
    data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
  int16_t vel_raw = static_cast<int16_t>(data[4] | (data[5] << 8));
  tpdo_position_[joint_idx] = static_cast<double>(pos_cnt) / kCountsPerRev * kTwoPi;
  tpdo_velocity_[joint_idx] = static_cast<double>(vel_raw) * kVelocityScale;
}

void CanopenSystem::decode_tpdo2(size_t joint_idx, const uint8_t * data)
{
  int16_t tau_raw = static_cast<int16_t>(data[2] | (data[3] << 8));
  tpdo_torque_[joint_idx] = static_cast<double>(tau_raw) * kTorqueScale;
}

void CanopenSystem::can_rx_loop()
{
  while (running_.load()) {
    struct can_frame frame {};
    const ssize_t n = ::read(can_socket_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      continue;
    }

    const uint32_t id = frame.can_id & CAN_EFF_MASK;
    std::lock_guard<std::mutex> lock(tpdo_mutex_);
    for (size_t i = 0; i < node_ids_.size(); ++i) {
      const uint8_t node_id = node_ids_[i];
      if (id == kTpdo1Base + node_id) {
        decode_tpdo1(i, frame.data);
        tpdo_received_ = true;
      } else if (id == kTpdo2Base + node_id) {
        decode_tpdo2(i, frame.data);
      }
    }
  }
}

hardware_interface::CallbackReturn CanopenSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = std::make_shared<rclcpp::Node>("canopen_hw_backplane");
  sub_estop_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/safety/estop", rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) { estop_active_.store(msg->data); });

  running_.store(true);

  if (use_sim_) {
    pub_sim_effort_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/sim/joint_effort_cmd", rclcpp::SensorDataQoS());
    sub_sim_encoder_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/sim/encoder_state", rclcpp::SensorDataQoS(),
      std::bind(&CanopenSystem::on_encoder_state, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "CanopenSystem activated in SIM mode (/sim backplane).");
  } else {
    if (!open_can_socket()) {
      running_.store(false);
      return hardware_interface::CallbackReturn::ERROR;
    }
    can_rx_thread_ = std::thread([this]() { can_rx_loop(); });
    ds402_enable_all();
    RCLCPP_INFO(
      get_logger(), "CanopenSystem activated in CAN mode on '%s'.", can_interface_.c_str());
  }

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });
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
    if (can_rx_thread_.joinable()) {
      can_rx_thread_.join();
    }
  }
  close_can_socket();
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
  if (use_sim_) {
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    if (encoder_received_) {
      hw_state_position_ = encoder_position_;
      hw_state_velocity_ = encoder_velocity_;
      hw_state_effort_ = encoder_effort_;
    }
  } else {
    std::lock_guard<std::mutex> lock(tpdo_mutex_);
    if (tpdo_received_) {
      hw_state_position_ = tpdo_position_;
      hw_state_velocity_ = tpdo_velocity_;
      hw_state_effort_ = tpdo_torque_;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanopenSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const bool estop = estop_active_.load();

  if (use_sim_) {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
      cmd.data[i] = estop ? 0.0 : hw_cmd_effort_[i];
    }
    if (pub_sim_effort_) {
      pub_sim_effort_->publish(cmd);
    }
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < num_joints_; ++i) {
    const double torque = estop ? 0.0 : hw_cmd_effort_[i];
    const auto payload = encode_rpdo_torque(torque);
    send_can_frame(kRpdoBase + node_ids_[i], payload.data(), static_cast<uint8_t>(payload.size()));
  }
  send_sync_frame();
  return hardware_interface::return_type::OK;
}

}  // namespace canopen_hw_interface

PLUGINLIB_EXPORT_CLASS(
  canopen_hw_interface::CanopenSystem, hardware_interface::SystemInterface)
