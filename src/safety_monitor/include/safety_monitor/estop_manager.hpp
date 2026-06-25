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

#ifndef SAFETY_MONITOR__ESTOP_MANAGER_HPP_
#define SAFETY_MONITOR__ESTOP_MANAGER_HPP_

#include <atomic>
#include <mutex>
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
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      active_ = true;
      reason_ = reason;
    }
  }

  /// Reset succeeds only if currently safe (no live fault).
  bool reset(bool currently_safe)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (currently_safe && active_) {
      active_ = false;
      reason_.clear();
      return true;
    }
    return false;
  }

  bool active() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
  }

  std::string reason() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return reason_;
  }

private:
  mutable std::mutex mutex_;
  bool active_{false};
  std::string reason_;
};

}  // namespace safety_monitor

#endif  // SAFETY_MONITOR__ESTOP_MANAGER_HPP_
