#ifndef SAFETY_MONITOR__WORKSPACE_LIMIT_MONITOR_HPP_
#define SAFETY_MONITOR__WORKSPACE_LIMIT_MONITOR_HPP_

#include <array>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace safety_monitor
{

/// Rejects commanded end-effector poses outside an axis-aligned Cartesian box.
class WorkspaceLimitMonitor
{
public:
  void configure(const std::array<double, 3> & min_xyz, const std::array<double, 3> & max_xyz)
  {
    min_ = min_xyz;
    max_ = max_xyz;
  }

  bool check(const geometry_msgs::msg::PoseStamped & pose, std::string & fault) const
  {
    const auto & p = pose.pose.position;
    const std::array<double, 3> xyz{p.x, p.y, p.z};
    const char * axis[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      if (xyz[i] < min_[i] || xyz[i] > max_[i]) {
        fault = std::string("workspace:") + axis[i];
        return false;
      }
    }
    return true;
  }

private:
  std::array<double, 3> min_{{-1.0, -1.0, 0.0}};
  std::array<double, 3> max_{{1.0, 1.0, 1.5}};
};

}  // namespace safety_monitor

#endif  // SAFETY_MONITOR__WORKSPACE_LIMIT_MONITOR_HPP_
