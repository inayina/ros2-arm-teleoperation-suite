#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
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
    // ---- Parameters ----
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
    const double watchdog_timeout = declare_parameter<double>("watchdog_timeout", 0.1);
    auto ws_min = declare_parameter<std::vector<double>>("workspace_min", {-0.8, -0.8, 0.0});
    auto ws_max = declare_parameter<std::vector<double>>("workspace_max", {0.8, 0.8, 1.3});

    joint_limit_.configure(joint_names_, lower, upper, margin);
    velocity_.configure(joint_names_, max_vel);
    workspace_.configure(
      {ws_min[0], ws_min[1], ws_min[2]}, {ws_max[0], ws_max[1], ws_max[2]});
    watchdog_.configure(watchdog_timeout);

    // ---- Pub/Sub ----
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
      [this](const std_msgs::msg::Header::SharedPtr) {watchdog_.on_heartbeat(now_s());});
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&SafetyMonitorNode::on_joint_states, this, std::placeholders::_1));

    // ---- Services ----
    srv_estop_ = create_service<teleop_interfaces::srv::TriggerEstop>(
      "/safety/trigger_estop",
      std::bind(&SafetyMonitorNode::on_trigger_estop, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_reset_ = create_service<std_srvs::srv::Trigger>(
      "/safety/reset",
      std::bind(&SafetyMonitorNode::on_reset, this,
        std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(4ms, std::bind(&SafetyMonitorNode::on_timer, this));  // 250 Hz
    RCLCPP_INFO(get_logger(), "safety_monitor up (250 Hz).");
  }

private:
  double now_s() {return this->get_clock()->now().seconds();}

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_js_ = *msg;
    have_js_ = true;
    watchdog_.on_joint_states(now_s());

    std::string fault;
    bool estop = false;
    if (!joint_limit_.check(*msg, fault)) {add_fault(fault);}
    if (!velocity_.check(*msg, fault, estop)) {
      add_fault(fault);
      if (estop) {estop_.trip(fault);}
    }
  }

  void on_cmd_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::string fault;
    bool pass = true;
    if (!workspace_.check(*msg, fault)) {add_fault(fault); pass = false;}

    if (pass && !estop_.active()) {
      last_safe_pose_ = *msg;
      have_safe_pose_ = true;
      pub_safe_pose_->publish(*msg);
    } else if (have_safe_pose_) {
      // Reject dangerous command: hold the last safe pose.
      pub_safe_pose_->publish(last_safe_pose_);
    }
  }

  void on_trigger_estop(
    const std::shared_ptr<teleop_interfaces::srv::TriggerEstop::Request> req,
    std::shared_ptr<teleop_interfaces::srv::TriggerEstop::Response> res)
  {
    estop_.trip("manual:" + req->reason);
    RCLCPP_WARN(get_logger(), "E-Stop tripped manually: %s", req->reason.c_str());
    res->success = true;
    res->message = "E-Stop active";
  }

  void on_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    const bool safe = live_faults_.empty();
    res->success = estop_.reset(safe);
    res->message = res->success ? "E-Stop reset" : "Cannot reset: faults present";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void add_fault(const std::string & f) {pending_faults_.push_back(f);}

  void on_timer()
  {
    // Watchdog evaluation.
    std::string fault;
    const bool comm_ok = watchdog_.ok(now_s(), fault);
    if (!comm_ok) {
      pending_faults_.push_back(fault);
      estop_.trip(fault);
    }

    live_faults_ = pending_faults_;
    pending_faults_.clear();

    publish_status(comm_ok);
    publish_estop();
  }

  void publish_estop()
  {
    std_msgs::msg::Bool m;
    m.data = estop_.active();
    pub_estop_->publish(m);
  }

  void publish_status(bool comm_ok)
  {
    teleop_interfaces::msg::SafetyStatus s;
    s.header.stamp = now();
    s.comm_ok = comm_ok;
    s.estop_active = estop_.active();

    bool jl = true, ws = true, vel = true;
    for (const auto & f : live_faults_) {
      if (f.rfind("joint_limit", 0) == 0) {jl = false;}
      else if (f.rfind("workspace", 0) == 0) {ws = false;}
      else if (f.rfind("velocity", 0) == 0) {vel = false;}
    }
    s.joint_limit_ok = jl;
    s.workspace_ok = ws;
    s.velocity_ok = vel;
    s.active_faults = live_faults_;
    s.ok = jl && ws && vel && comm_ok && !estop_.active();
    s.level = s.estop_active ? s.LEVEL_ESTOP
      : (s.ok ? s.LEVEL_OK : s.LEVEL_ERROR);
    pub_status_->publish(s);

    diagnostic_msgs::msg::DiagnosticArray da;
    da.header.stamp = s.header.stamp;
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.name = "safety_monitor";
    ds.hardware_id = "teleop_arm";
    ds.level = s.estop_active ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
      : (s.ok ? diagnostic_msgs::msg::DiagnosticStatus::OK
              : diagnostic_msgs::msg::DiagnosticStatus::WARN);
    ds.message = s.estop_active ? ("E-STOP: " + estop_.reason())
      : (s.ok ? "OK" : "fault");
    da.status.push_back(ds);
    pub_diag_->publish(da);
  }

  // Monitors
  JointLimitMonitor joint_limit_;
  WorkspaceLimitMonitor workspace_;
  VelocityLimitMonitor velocity_;
  CommWatchdog watchdog_;
  EstopManager estop_;

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
