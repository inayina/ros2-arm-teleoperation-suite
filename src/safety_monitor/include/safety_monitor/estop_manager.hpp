#ifndef SAFETY_MONITOR__ESTOP_MANAGER_HPP_
#define SAFETY_MONITOR__ESTOP_MANAGER_HPP_

#include <string>

namespace safety_monitor
{

/// Latches the emergency-stop state. Once tripped it stays active until an
/// explicit reset (and only if no fault is currently asserted).
class EstopManager
{
public:
  void trip(const std::string & reason)
  {
    if (!active_) {
      active_ = true;
      reason_ = reason;
    }
  }

  /// Reset succeeds only if currently safe (no live fault).
  bool reset(bool currently_safe)
  {
    if (currently_safe) {
      active_ = false;
      reason_.clear();
      return true;
    }
    return false;
  }

  bool active() const {return active_;}
  const std::string & reason() const {return reason_;}

private:
  bool active_{false};
  std::string reason_;
};

}  // namespace safety_monitor

#endif  // SAFETY_MONITOR__ESTOP_MANAGER_HPP_
