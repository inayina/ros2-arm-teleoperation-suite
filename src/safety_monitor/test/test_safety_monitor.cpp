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

#include <gtest/gtest.h>

#include "safety_monitor/joint_limit_monitor.hpp"
#include "safety_monitor/workspace_limit_monitor.hpp"
#include "safety_monitor/velocity_limit_monitor.hpp"
#include "safety_monitor/comm_watchdog.hpp"
#include "safety_monitor/estop_manager.hpp"

TEST(JointLimitMonitor, RejectsOverLimit)
{
  safety_monitor::JointLimitMonitor m;
  m.configure({"j1"}, {-1.0}, {1.0}, 0.05);
  sensor_msgs::msg::JointState js;
  std::string fault;

  js.name = {"j1"};
  js.position = {0.0};
  EXPECT_TRUE(m.check(js, fault));

  js.position = {0.99};  // within margin band -> reject
  EXPECT_FALSE(m.check(js, fault));
  EXPECT_EQ(fault, "joint_limit:j1");
}

TEST(JointLimitMonitor, MatchesByJointName)
{
  safety_monitor::JointLimitMonitor m;
  m.configure({"j1"}, {-1.0}, {1.0}, 0.05);
  sensor_msgs::msg::JointState js;
  std::string fault;

  js.name = {"j2", "j1"};
  js.position = {0.0, 0.99};
  EXPECT_FALSE(m.check(js, fault));
  EXPECT_EQ(fault, "joint_limit:j1");
}

TEST(WorkspaceLimitMonitor, RejectsOutOfBox)
{
  safety_monitor::WorkspaceLimitMonitor m;
  m.configure({-0.5, -0.5, 0.0}, {0.5, 0.5, 1.0});
  geometry_msgs::msg::PoseStamped p;
  std::string fault;

  p.pose.position.x = 0.0; p.pose.position.y = 0.0; p.pose.position.z = 0.5;
  EXPECT_TRUE(m.check(p, fault));

  p.pose.position.z = 1.5;  // above box
  EXPECT_FALSE(m.check(p, fault));
  EXPECT_EQ(fault, "workspace:z");
}

TEST(VelocityLimitMonitor, FlagsAndEstops)
{
  safety_monitor::VelocityLimitMonitor m;
  m.configure({"j1"}, {1.0}, 1.5);
  sensor_msgs::msg::JointState js;
  std::string fault;
  bool estop = false;

  js.name = {"j1"};
  js.velocity = {0.5};
  EXPECT_TRUE(m.check(js, fault, estop));
  EXPECT_FALSE(estop);

  js.velocity = {1.2};  // mild excess -> fault, no estop
  EXPECT_FALSE(m.check(js, fault, estop));
  EXPECT_FALSE(estop);

  js.velocity = {2.0};  // severe -> estop
  EXPECT_FALSE(m.check(js, fault, estop));
  EXPECT_TRUE(estop);
}

TEST(CommWatchdog, TimesOut)
{
  safety_monitor::CommWatchdog w;
  w.configure(0.1, 0.2);
  std::string fault;

  EXPECT_TRUE(w.ok(0.0, fault));  // startup grace before first heartbeat

  w.on_heartbeat(1.0);
  w.on_joint_states(1.0);
  EXPECT_TRUE(w.ok(1.05, fault));
  EXPECT_FALSE(w.ok(1.2, fault));  // heartbeat stale (>100ms)
}

TEST(EstopManager, LatchesUntilSafeReset)
{
  safety_monitor::EstopManager e;
  EXPECT_FALSE(e.active());
  e.trip("velocity:j1");
  EXPECT_TRUE(e.active());
  EXPECT_FALSE(e.reset(false));  // faults present -> cannot reset
  EXPECT_TRUE(e.active());
  EXPECT_TRUE(e.reset(true));    // safe -> reset
  EXPECT_FALSE(e.active());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
