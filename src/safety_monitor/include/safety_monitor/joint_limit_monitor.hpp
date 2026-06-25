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

#ifndef SAFETY_MONITOR__JOINT_LIMIT_MONITOR_HPP_
#define SAFETY_MONITOR__JOINT_LIMIT_MONITOR_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

#include "safety_monitor/joint_state_utils.hpp"

namespace safety_monitor
{

/// Rejects commands when any joint is at/over its (soft) position limit.
class JointLimitMonitor
{
public:
  void configure(
    const std::vector<std::string> & names,
    const std::vector<double> & lower,
    const std::vector<double> & upper,
    double margin)
  {
    names_ = names;
    lower_ = lower;
    upper_ = upper;
    margin_ = margin;
  }

  /// Returns true if all measured joints are within soft limits.
  bool check(const sensor_msgs::msg::JointState & js, std::string & fault) const
  {
    const size_t n = std::min({names_.size(), lower_.size(), upper_.size()});
    for (size_t i = 0; i < n; ++i) {
      double q = 0.0;
      if (!joint_value(js, names_[i], js.position, q)) {
        continue;
      }
      if (q < lower_[i] + margin_ || q > upper_[i] - margin_) {
        fault = "joint_limit:" + names_[i];
        return false;
      }
    }
    return true;
  }

private:
  std::vector<std::string> names_;
  std::vector<double> lower_;
  std::vector<double> upper_;
  double margin_{0.0};
};

}  // namespace safety_monitor

#endif  // SAFETY_MONITOR__JOINT_LIMIT_MONITOR_HPP_
