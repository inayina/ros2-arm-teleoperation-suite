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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef TELEOP_CONTROLLERS__JOINT_TARGET_GUARD_HPP_
#define TELEOP_CONTROLLERS__JOINT_TARGET_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace teleop_controllers
{

/// Reject malformed or discontinuous Servo targets before they reach the
/// impedance controller. Normal Servo output advances incrementally from the
/// measured state; a large one-message excursion indicates stale startup data
/// or a frame/order mismatch.
inline bool joint_target_within_excursion(
  const std::vector<double> & target,
  const std::vector<double> & measured,
  double max_excursion_rad,
  double & observed_excursion_rad)
{
  observed_excursion_rad = std::numeric_limits<double>::infinity();
  if (target.empty() || target.size() != measured.size() ||
    !std::isfinite(max_excursion_rad) || max_excursion_rad <= 0.0)
  {
    return false;
  }

  double observed = 0.0;
  for (size_t i = 0; i < target.size(); ++i) {
    if (!std::isfinite(target[i]) || !std::isfinite(measured[i])) {
      return false;
    }
    observed = std::max(observed, std::abs(target[i] - measured[i]));
  }
  observed_excursion_rad = observed;
  return observed <= max_excursion_rad;
}

}  // namespace teleop_controllers

#endif  // TELEOP_CONTROLLERS__JOINT_TARGET_GUARD_HPP_
