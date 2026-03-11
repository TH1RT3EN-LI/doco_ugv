#include <gtest/gtest.h>

#include "ugv_teleop/joy_launcher_logic.hpp"

TEST(JoyLauncherLogic, ParsesBindingsAndWarnings)
{
  std::vector<std::string> warnings;
  const auto parsed = ugv_teleop::parse_button_commands(
    {"1:echo one", "bad", "2:echo two"},
    &warnings);

  ASSERT_EQ(parsed.size(), 2U);
  EXPECT_EQ(parsed.at(1), "echo one");
  EXPECT_EQ(parsed.at(2), "echo two");
  ASSERT_EQ(warnings.size(), 1U);
}

TEST(JoyLauncherLogic, HoldSecondsFallbackToDefault)
{
  const std::unordered_map<int, double> hold_secs{{3, 2.5}};
  EXPECT_DOUBLE_EQ(ugv_teleop::hold_seconds_for_button(3, hold_secs, 1.0), 2.5);
  EXPECT_DOUBLE_EQ(ugv_teleop::hold_seconds_for_button(7, hold_secs, 1.0), 1.0);
}

TEST(JoyLauncherLogic, HoldTrackerTriggersOnlyOncePerPress)
{
  ugv_teleop::ButtonHoldTracker tracker;

  auto result = tracker.update(1, true, false, 0.0, 1.0);
  EXPECT_FALSE(result.triggered);

  result = tracker.update(1, true, true, 1.2, 1.0);
  EXPECT_TRUE(result.triggered);
  EXPECT_DOUBLE_EQ(result.held_sec, 1.2);

  result = tracker.update(1, true, true, 1.5, 1.0);
  EXPECT_FALSE(result.triggered);

  result = tracker.update(1, false, true, 1.6, 1.0);
  EXPECT_FALSE(result.triggered);

  result = tracker.update(1, true, false, 2.0, 1.0);
  EXPECT_FALSE(result.triggered);
}
