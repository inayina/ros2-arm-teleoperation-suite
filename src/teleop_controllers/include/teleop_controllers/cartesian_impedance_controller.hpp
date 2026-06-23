#ifndef TELEOP_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_
#define TELEOP_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace teleop_controllers
{

/// Cartesian impedance controller (ros2_control plugin).
///
/// Scaffold computes a joint-space PD law as a stand-in; the M3 task is to
/// replace it with the full Cartesian law:
///   tau = J^T [ K (x_d - x) + D (xdot_d - xdot) ] + g(q)
/// with contact-adaptive stiffness from /ft_sensor.
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
  std::vector<std::string> joint_names_;
  size_t num_joints_{0};

  // Joint-space gains (scaffold). Cartesian K/D added in M3.
  std::vector<double> kp_;
  std::vector<double> kd_;

  // Reference target from /joint_target (RT-safe).
  realtime_tools::RealtimeBuffer<std::vector<double>> target_positions_;

  // Latest contact wrench from /ft_sensor.
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::WrenchStamped> ft_buffer_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_target_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_ft_;
};

}  // namespace teleop_controllers

#endif  // TELEOP_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_HPP_
