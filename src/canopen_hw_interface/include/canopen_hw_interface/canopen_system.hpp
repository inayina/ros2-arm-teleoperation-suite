#ifndef CANOPEN_HW_INTERFACE__CANOPEN_SYSTEM_HPP_
#define CANOPEN_HW_INTERFACE__CANOPEN_SYSTEM_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace canopen_hw_interface
{

/// ros2_control SystemInterface bridging joint interfaces to a CANopen DS402 bus.
///
/// Two backends, selected by the `use_sim` hardware parameter:
///   * use_sim=true  -> /sim backplane to mujoco_sim (publish effort, read encoders)
///   * use_sim=false -> SocketCAN (vcan0/can0): RPDO write + TPDO read
class CanopenSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Config
  bool use_sim_{true};
  std::string can_interface_{"vcan0"};
  size_t num_joints_{0};
  std::vector<uint8_t> node_ids_;

  // Joint storage (index aligned with info_.joints)
  std::vector<double> hw_cmd_effort_;
  std::vector<double> hw_state_position_;
  std::vector<double> hw_state_velocity_;
  std::vector<double> hw_state_effort_;

  // E-stop latched flag (set by /safety/estop -> DS402 Quick Stop)
  std::atomic<bool> estop_active_{false};

  // ---- sim backplane (use_sim=true) ----
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_sim_effort_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_sim_encoder_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  // Latest encoder feedback from sim, guarded for the read() thread.
  std::mutex encoder_mutex_;
  std::vector<double> encoder_position_;
  std::vector<double> encoder_velocity_;
  std::vector<double> encoder_effort_;
  bool encoder_received_{false};

  void on_encoder_state(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ---- SocketCAN (use_sim=false) ----
  int can_socket_{-1};
  std::thread can_rx_thread_;
  std::mutex tpdo_mutex_;
  std::vector<double> tpdo_position_;
  std::vector<double> tpdo_velocity_;
  std::vector<double> tpdo_torque_;
  bool tpdo_received_{false};

  bool open_can_socket();
  void close_can_socket();
  void can_rx_loop();
  bool send_can_frame(uint32_t cob_id, const uint8_t * data, uint8_t dlc);
  bool send_sync_frame();
  bool send_nmt_start();
  bool sdo_write_u16(uint8_t node_id, uint16_t index, uint16_t value);
  void ds402_enable_all();
  void decode_tpdo1(size_t joint_idx, const uint8_t * data);
  void decode_tpdo2(size_t joint_idx, const uint8_t * data);
  static std::vector<uint8_t> encode_rpdo_torque(double torque_nm);
};

}  // namespace canopen_hw_interface

#endif  // CANOPEN_HW_INTERFACE__CANOPEN_SYSTEM_HPP_
