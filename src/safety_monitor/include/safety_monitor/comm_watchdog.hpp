#ifndef SAFETY_MONITOR__COMM_WATCHDOG_HPP_
#define SAFETY_MONITOR__COMM_WATCHDOG_HPP_

#include <string>

namespace safety_monitor
{

/// Watches freshness of teleop heartbeat and joint_states.
/// Times are in seconds (use a monotonic clock from the caller).
class CommWatchdog
{
public:
  void configure(double timeout_s) {timeout_s_ = timeout_s;}

  void on_heartbeat(double now_s) {last_heartbeat_s_ = now_s;}
  void on_joint_states(double now_s) {last_joint_states_s_ = now_s;}

  /// false if either stream is stale beyond timeout (=> e-stop).
  bool ok(double now_s, std::string & fault) const
  {
    if (last_heartbeat_s_ < 0.0 || (now_s - last_heartbeat_s_) > timeout_s_) {
      fault = "watchdog:teleop";
      return false;
    }
    if (last_joint_states_s_ < 0.0 || (now_s - last_joint_states_s_) > timeout_s_) {
      fault = "watchdog:joint_states";
      return false;
    }
    return true;
  }

private:
  double timeout_s_{0.1};
  double last_heartbeat_s_{-1.0};
  double last_joint_states_s_{-1.0};
};

}  // namespace safety_monitor

#endif  // SAFETY_MONITOR__COMM_WATCHDOG_HPP_
