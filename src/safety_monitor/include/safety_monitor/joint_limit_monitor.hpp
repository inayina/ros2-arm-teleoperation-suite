#ifndef SAFETY_MONITOR__JOINT_LIMIT_MONITOR_HPP_
#define SAFETY_MONITOR__JOINT_LIMIT_MONITOR_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

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
    const size_t n = std::min({names_.size(), lower_.size(), upper_.size(), js.position.size()});
    for (size_t i = 0; i < n; ++i) {
      const double q = js.position[i];
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
