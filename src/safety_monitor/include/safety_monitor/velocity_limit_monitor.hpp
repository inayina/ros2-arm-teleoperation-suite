#ifndef SAFETY_MONITOR__VELOCITY_LIMIT_MONITOR_HPP_
#define SAFETY_MONITOR__VELOCITY_LIMIT_MONITOR_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

namespace safety_monitor
{

/// Flags joint over-speed. Mild excess -> WARN (clamp upstream); large -> E-Stop.
class VelocityLimitMonitor
{
public:
  void configure(
    const std::vector<std::string> & names,
    const std::vector<double> & max_velocity,
    double estop_factor = 1.5)
  {
    names_ = names;
    max_vel_ = max_velocity;
    estop_factor_ = estop_factor;
  }

  /// out_estop set true when excess is severe enough to demand an e-stop.
  bool check(const sensor_msgs::msg::JointState & js, std::string & fault, bool & out_estop) const
  {
    out_estop = false;
    const size_t n = std::min({names_.size(), max_vel_.size(), js.velocity.size()});
    for (size_t i = 0; i < n; ++i) {
      const double v = std::abs(js.velocity[i]);
      if (v > max_vel_[i]) {
        fault = "velocity:" + names_[i];
        if (v > estop_factor_ * max_vel_[i]) {out_estop = true;}
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
