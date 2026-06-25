// Copyright 2026 ros2-arm-teleoperation-suite contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "teleop_interfaces/msg/safety_status.hpp"
#include "teleop_interfaces/srv/trigger_estop.hpp"

#include "safety_monitor/joint_limit_monitor.hpp"
#include "safety_monitor/workspace_limit_monitor.hpp"
#include "safety_monitor/velocity_limit_monitor.hpp"
#include "safety_monitor/comm_watchdog.hpp"
#include "safety_monitor/estop_manager.hpp"

using namespace std::chrono_literals;

namespace safety_monitor
{

class SafetyMonitorNode : public rclcpp::Node
{
public:
  SafetyMonitorNode()
  : rclcpp::Node("safety_monitor")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joints", {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7"});
    auto lower = declare_parameter<std::vector<double>>(
      "lower", {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973});
    auto upper = declare_parameter<std::vector<double>>(
      "upper", {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973});
    auto max_vel = declare_parameter<std::vector<double>>(
      "max_velocity", {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61});
    const double margin = declare_parameter<double>("soft_limit_margin", 0.05);
    const double heartbeat_timeout = declare_parameter<double>("watchdog_timeout", 0.1);
    const double joint_states_timeout = declare_parameter<double>("joint_states_timeout", 0.2);
    auto_estop_enabled_ = declare_parameter<bool>("auto_estop_enabled", true);
    velocity_auto_estop_enabled_ = declare_parameter<bool>("velocity_auto_estop_enabled", false);
    startup_grace_s_ = declare_parameter<double>("startup_grace_s", 8.0);
    auto ws_min = declare_parameter<std::vector<double>>("workspace_min", {-0.8, -0.8, 0.0});
    auto ws_max = declare_parameter<std::vector<double>>("workspace_max", {0.8, 0.8, 1.3});

    joint_limit_.configure(joint_names_, lower, upper, margin);
    velocity_.configure(joint_names_, max_vel);
    workspace_.configure(
      {ws_min[0], ws_min[1], ws_min[2]}, {ws_max[0], ws_max[1], ws_max[2]});
    watchdog_.configure(heartbeat_timeout, joint_states_timeout);

    auto estop_qos = rclcpp::QoS(1).reliable().transient_local();
    pub_safe_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/safe_master_pose", rclcpp::QoS(10).reliable());
    pub_estop_ = create_publisher<std_msgs::msg::Bool>("/safety/estop", estop_qos);
    pub_status_ = create_publisher<teleop_interfaces::msg::SafetyStatus>(
      "/safety/status", rclcpp::QoS(10).reliable());
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/safety/diagnostics", rclcpp::QoS(10).reliable());

    sub_cmd_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/teleop/cmd_pose", rclcpp::QoS(10),
      std::bind(&SafetyMonitorNode::on_cmd_pose, this, std::placeholders::_1));
    sub_hb_ = create_subscription<std_msgs::msg::Header>(
      "/teleop/heartbeat", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Header::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        watchdog_.on_heartbeat(now_s());
      });
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&SafetyMonitorNode::on_joint_states, this, std::placeholders::_1));

    srv_estop_ = create_service<teleop_interfaces::srv::TriggerEstop>(
      "/safety/trigger_estop",
      std::bind(&SafetyMonitorNode::on_trigger_estop, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_reset_ = create_service<std_srvs::srv::Trigger>(
      "/safety/reset",
      std::bind(&SafetyMonitorNode::on_reset, this,
        std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(4ms, std::bind(&SafetyMonitorNode::on_timer, this));
    RCLCPP_INFO(
      get_logger(), "safety_monitor up (250 Hz, auto_estop=%s).",
      auto_estop_enabled_ ? "true" : "false");
  }

private:
  double now_s() const {return this->get_clock()->now().seconds();}

  void add_fault_locked(const std::string & fault)
  {
    pending_faults_.push_back(fault);
  }

  void trip_estop_locked(const std::string & reason)
  {
    if (!estop_.active()) {
      estop_.trip(reason);
      publish_estop_locked();
      RCLCPP_ERROR(get_logger(), "E-STOP: %s", reason.c_str());
    }
  }

  bool velocity_estop_allowed_locked() const
  {
    return first_js_time_s_ < 0.0 ||
           (now_s() - first_js_time_s_) >= startup_grace_s_;
  }

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_js_) {
      first_js_time_s_ = now_s();
    }
    last_js_ = *msg;
    have_js_ = true;
    watchdog_.on_joint_states(now_s());

    std::string fault;
    bool estop_flag = false;
    if (!joint_limit_.check(*msg, fault)) {
      add_fault_locked(fault);
    }
    if (!velocity_.check(*msg, fault, estop_flag)) {
      add_fault_locked(fault);
      if (estop_flag && auto_estop_enabled_ && velocity_auto_estop_enabled_ &&
        velocity_estop_allowed_locked())
      {
        trip_estop_locked(fault);
      }
    }
  }

  bool monitors_pass_locked(std::string & fault_out)
  {
    if (estop_.active()) {
      fault_out = "estop:" + estop_.reason();
      return false;
    }

    std::string fault;
    if (!watchdog_.ok(now_s(), fault)) {
      fault_out = fault;
      add_fault_locked(fault);
      if (auto_estop_enabled_) {
        const bool js_watchdog = fault.rfind("watchdog:joint_states", 0) == 0;
        if (!js_watchdog || velocity_estop_allowed_locked()) {
          trip_estop_locked(fault);
        }
      }
      return false;
    }

    if (have_js_ && !joint_limit_.check(last_js_, fault)) {
      fault_out = fault;
      return false;
    }

    if (have_js_) {
      bool estop_flag = false;
      if (!velocity_.check(last_js_, fault, estop_flag)) {
        fault_out = fault;
        if (estop_flag && auto_estop_enabled_ && velocity_auto_estop_enabled_ &&
          velocity_estop_allowed_locked())
        {
          trip_estop_locked(fault);
        }
        return false;
      }
    }

    return true;
  }

  void on_cmd_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::string fault;
    bool pass = monitors_pass_locked(fault);

    if (pass && !workspace_.check(*msg, fault)) {
      add_fault_locked(fault);
      pass = false;
    }

    if (pass && !estop_.active()) {
      last_safe_pose_ = *msg;
      have_safe_pose_ = true;
      pub_safe_pose_->publish(*msg);
    } else if (have_safe_pose_) {
      pub_safe_pose_->publish(last_safe_pose_);
    }
  }

  void on_trigger_estop(
    const std::shared_ptr<teleop_interfaces::srv::TriggerEstop::Request> req,
    std::shared_ptr<teleop_interfaces::srv::TriggerEstop::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    trip_estop_locked("manual:" + req->reason);
    RCLCPP_WARN(get_logger(), "E-Stop tripped manually: %s", req->reason.c_str());
    res->success = true;
    res->message = "E-Stop active";
  }

  void on_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool safe = live_faults_.empty();
    res->success = estop_.reset(safe);
    if (res->success) {
      publish_estop_locked();
    }
    res->message = res->success ? "E-Stop reset" : "Cannot reset: faults present";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void on_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::string fault;
    if (!watchdog_.ok(now_s(), fault)) {
      add_fault_locked(fault);
      if (auto_estop_enabled_) {
        const bool js_watchdog = fault.rfind("watchdog:joint_states", 0) == 0;
        if (!js_watchdog || velocity_estop_allowed_locked()) {
          trip_estop_locked(fault);
        }
      }
    }

    live_faults_ = pending_faults_;
    pending_faults_.clear();

    publish_status_locked();
    publish_estop_locked();
  }

  void publish_estop_locked()
  {
    std_msgs::msg::Bool m;
    m.data = estop_.active();
    pub_estop_->publish(m);
  }

  static diagnostic_msgs::msg::DiagnosticStatus make_diag(
    const std::string & name, uint8_t level, const std::string & message)
  {
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.name = name;
    ds.hardware_id = "teleop_arm";
    ds.level = level;
    ds.message = message;
    return ds;
  }

  void publish_status_locked()
  {
    const bool comm_ok = [&]() {
        std::string fault;
        return watchdog_.ok(now_s(), fault);
      }();

    teleop_interfaces::msg::SafetyStatus s;
    s.header.stamp = now();
    s.comm_ok = comm_ok;
    s.estop_active = estop_.active();

    bool jl = true, ws = true, vel = true;
    for (const auto & f : live_faults_) {
      if (f.rfind("joint_limit", 0) == 0) {
        jl = false;
      } else if (f.rfind("workspace", 0) == 0) {
        ws = false;
      } else if (f.rfind("velocity", 0) == 0) {
        vel = false;
      }
    }
    s.joint_limit_ok = jl;
    s.workspace_ok = ws;
    s.velocity_ok = vel;
    s.active_faults = live_faults_;
    s.ok = jl && ws && vel && comm_ok && !estop_.active();
    s.level = s.estop_active ? s.LEVEL_ESTOP :
      (s.ok ? s.LEVEL_OK : s.LEVEL_ERROR);
    pub_status_->publish(s);

    diagnostic_msgs::msg::DiagnosticArray da;
    da.header.stamp = s.header.stamp;

    const auto ok_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    const auto warn_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    const auto err_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    da.status.push_back(make_diag(
      "safety_monitor/joint_limit", jl ? ok_level : warn_level,
      jl ? "OK" : "joint limit fault"));
    da.status.push_back(make_diag(
      "safety_monitor/workspace", ws ? ok_level : warn_level,
      ws ? "OK" : "workspace fault"));
    da.status.push_back(make_diag(
      "safety_monitor/velocity", vel ? ok_level : warn_level,
      vel ? "OK" : "velocity fault"));
    da.status.push_back(make_diag(
      "safety_monitor/comm_watchdog", comm_ok ? ok_level : err_level,
      comm_ok ? "OK" : "communication stale"));
    da.status.push_back(make_diag(
      "safety_monitor/estop", s.estop_active ? err_level : ok_level,
      s.estop_active ? ("latched: " + estop_.reason()) : "inactive"));

    pub_diag_->publish(da);
  }

  JointLimitMonitor joint_limit_;
  WorkspaceLimitMonitor workspace_;
  VelocityLimitMonitor velocity_;
  CommWatchdog watchdog_;
  EstopManager estop_;
  bool auto_estop_enabled_{true};
  bool velocity_auto_estop_enabled_{false};
  double startup_grace_s_{8.0};
  double first_js_time_s_{-1.0};

  std::mutex mutex_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> pending_faults_;
  std::vector<std::string> live_faults_;

  sensor_msgs::msg::JointState last_js_;
  bool have_js_{false};
  geometry_msgs::msg::PoseStamped last_safe_pose_;
  bool have_safe_pose_{false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_safe_pose_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_;
  rclcpp::Publisher<teleop_interfaces::msg::SafetyStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr sub_hb_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Service<teleop_interfaces::srv::TriggerEstop>::SharedPtr srv_estop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace safety_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<safety_monitor::SafetyMonitorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
