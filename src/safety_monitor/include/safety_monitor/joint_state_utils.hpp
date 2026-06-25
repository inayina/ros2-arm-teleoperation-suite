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

#ifndef SAFETY_MONITOR__JOINT_STATE_UTILS_HPP_
#define SAFETY_MONITOR__JOINT_STATE_UTILS_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

namespace safety_monitor
{

inline bool joint_value(
  const sensor_msgs::msg::JointState & js,
  const std::string & joint_name,
  const std::vector<double> & values,
  double & out)
{
  for (size_t i = 0; i < js.name.size(); ++i) {
    if (js.name[i] != joint_name) {
      continue;
    }
    if (i >= values.size()) {
      return false;
    }
    out = values[i];
    return true;
  }
  return false;
}

}  // namespace safety_monitor

#endif  // SAFETY_MONITOR__JOINT_STATE_UTILS_HPP_
