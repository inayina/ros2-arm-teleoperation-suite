#include <gtest/gtest.h>

#include "safety_monitor/joint_limit_monitor.hpp"
#include "safety_monitor/workspace_limit_monitor.hpp"
#include "safety_monitor/velocity_limit_monitor.hpp"
#include "safety_monitor/comm_watchdog.hpp"
#include "safety_monitor/estop_manager.hpp"

using namespace safety_monitor;

TEST(JointLimitMonitor, RejectsOverLimit)
{
  JointLimitMonitor m;
  m.configure({"j1"}, {-1.0}, {1.0}, 0.05);
  sensor_msgs::msg::JointState js;
  std::string fault;

  js.position = {0.0};
  EXPECT_TRUE(m.check(js, fault));

  js.position = {0.99};  // within margin band -> reject
  EXPECT_FALSE(m.check(js, fault));
  EXPECT_EQ(fault, "joint_limit:j1");
}

TEST(WorkspaceLimitMonitor, RejectsOutOfBox)
{
  WorkspaceLimitMonitor m;
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
  VelocityLimitMonitor m;
  m.configure({"j1"}, {1.0}, 1.5);
  sensor_msgs::msg::JointState js;
  std::string fault;
  bool estop = false;

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
  CommWatchdog w;
  w.configure(0.1);
  std::string fault;

  EXPECT_FALSE(w.ok(0.0, fault));  // nothing received yet

  w.on_heartbeat(1.0);
  w.on_joint_states(1.0);
  EXPECT_TRUE(w.ok(1.05, fault));
  EXPECT_FALSE(w.ok(1.2, fault));  // stale
}

TEST(EstopManager, LatchesUntilSafeReset)
{
  EstopManager e;
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
