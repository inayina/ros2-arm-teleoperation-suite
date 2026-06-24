// Copyright 2026 ros2-arm-teleoperation-suite contributors
// SPDX-License-Identifier: MIT
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef TELEOP_CONTROLLERS__JOINT_TRAJECTORY_MAPPING_HPP_
#define TELEOP_CONTROLLERS__JOINT_TRAJECTORY_MAPPING_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace teleop_controllers
{

/// Map the latest JointTrajectory point into the controller's configured joint order.
inline bool map_joint_trajectory_target(
  const trajectory_msgs::msg::JointTrajectory & msg,
  const std::vector<std::string> & controller_joint_names,
  std::vector<double> & mapped_positions)
{
  if (msg.points.empty() || controller_joint_names.empty()) {
    return false;
  }

  const auto & positions = msg.points.back().positions;
  if (msg.joint_names.empty()) {
    if (positions.size() != controller_joint_names.size()) {
      return false;
    }
    mapped_positions.assign(positions.begin(), positions.end());
    return true;
  }

  if (msg.joint_names.size() != positions.size() ||
    msg.joint_names.size() != controller_joint_names.size())
  {
    return false;
  }

  mapped_positions.assign(controller_joint_names.size(), 0.0);
  for (const auto & name : msg.joint_names) {
    if (std::find(controller_joint_names.begin(), controller_joint_names.end(), name) ==
      controller_joint_names.end())
    {
      return false;
    }
  }
  for (size_t i = 0; i < controller_joint_names.size(); ++i) {
    const auto it = std::find(msg.joint_names.begin(), msg.joint_names.end(),
        controller_joint_names[i]);
    if (it == msg.joint_names.end()) {
      return false;
    }
    mapped_positions[i] = positions[std::distance(msg.joint_names.begin(), it)];
  }
  return true;
}

}  // namespace teleop_controllers

#endif  // TELEOP_CONTROLLERS__JOINT_TRAJECTORY_MAPPING_HPP_
