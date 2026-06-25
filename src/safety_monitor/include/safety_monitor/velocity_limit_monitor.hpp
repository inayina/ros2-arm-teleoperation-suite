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

#ifndef SAFETY_MONITOR__VELOCITY_LIMIT_MONITOR_HPP_
#define SAFETY_MONITOR__VELOCITY_LIMIT_MONITOR_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

#include "safety_monitor/joint_state_utils.hpp"

namespace safety_monitor
{

class VelocityLimitMonitor
{
public:
  void configure(
    const std::vector<std::string> & names,
    const std::vector<double> & max_vel,
    double estop_factor = 1.5)
  {
    names_ = names;
    max_vel_ = max_vel;
    estop_factor_ = estop_factor;
  }

  /// out_estop set true when excess is severe enough to demand an e-stop.
  bool check(const sensor_msgs::msg::JointState & js, std::string & fault, bool & out_estop) const
  {
    out_estop = false;
    const size_t n = std::min(names_.size(), max_vel_.size());
    for (size_t i = 0; i < n; ++i) {
      double v = 0.0;
      if (!joint_value(js, names_[i], js.velocity, v)) {
        continue;
      }
      v = std::abs(v);
      if (v > max_vel_[i]) {
        fault = "velocity:" + names_[i];
        if (v > estop_factor_ * max_vel_[i]) {
          out_estop = true;
        }
        return false;
      }
    }
    return true;
  }

private:
  std::vector<std::string> names_;
  std::vector<double> max_vel_;
  double estop_factor_{1.5};
};

}  // namespace safety_monitor

#endif  // SAFETY_MONITOR__VELOCITY_LIMIT_MONITOR_HPP_
