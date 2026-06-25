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
  void configure(double heartbeat_timeout_s, double joint_states_timeout_s)
  {
    heartbeat_timeout_s_ = heartbeat_timeout_s;
    joint_states_timeout_s_ = joint_states_timeout_s;
  }

  void on_heartbeat(double now_s) {last_heartbeat_s_ = now_s;}
  void on_joint_states(double now_s) {last_joint_states_s_ = now_s;}

  /// false if either stream is stale beyond timeout (=> e-stop when enabled).
  /// Unreceived streams are ignored until the first message arrives (startup grace).
  bool ok(double now_s, std::string & fault) const
  {
    if (last_heartbeat_s_ >= 0.0 &&
      (now_s - last_heartbeat_s_) > heartbeat_timeout_s_)
    {
      fault = "watchdog:teleop";
      return false;
    }
    if (last_joint_states_s_ >= 0.0 &&
      (now_s - last_joint_states_s_) > joint_states_timeout_s_)
    {
      fault = "watchdog:joint_states";
      return false;
    }
    return true;
  }

private:
  double heartbeat_timeout_s_{0.1};
  double joint_states_timeout_s_{0.2};
  double last_heartbeat_s_{-1.0};
  double last_joint_states_s_{-1.0};
};

}  // namespace safety_monitor

#endif  // SAFETY_MONITOR__COMM_WATCHDOG_HPP_
