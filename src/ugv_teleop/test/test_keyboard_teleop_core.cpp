#include <chrono>

#include <gtest/gtest.h>

#include "ugv_teleop/keyboard_teleop_core.hpp"

namespace
{

using Clock = std::chrono::steady_clock;

ugv_teleop::KeyboardTeleopCore create_core(double idle_timeout_sec)
{
  return ugv_teleop::KeyboardTeleopCore(
    1.0,
    1.0,
    0.1,
    0.1,
    100.0,
    100.0,
    100.0,
    100.0,
    idle_timeout_sec);
}

}  // namespace

TEST(KeyboardTeleopCore, HoldPersistsWithoutRepeatedInputWhenIdleTimeoutDisabled)
{
  auto core = create_core(0.0);
  const auto start = Clock::now();

  EXPECT_TRUE(core.handle_key_press("w", start));

  const auto command = core.snapshot(start + std::chrono::milliseconds(500));
  EXPECT_DOUBLE_EQ(command.linear_x, 1.0);
  ASSERT_EQ(command.active_keys.size(), 1U);
  EXPECT_EQ(command.active_keys.front(), "w");
}

TEST(KeyboardTeleopCore, IdleTimeoutCanStillClearHeldKeysWhenExplicitlyConfigured)
{
  auto core = create_core(0.25);
  const auto start = Clock::now();

  EXPECT_TRUE(core.handle_key_press("w", start));

  const auto command = core.snapshot(start + std::chrono::milliseconds(300));
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_TRUE(command.active_keys.empty());
}

TEST(KeyboardTeleopCore, StaleTimeoutClearsSyntheticTtyHold)
{
  auto core = create_core(0.0);
  const auto start = Clock::now();

  EXPECT_TRUE(core.handle_key_press("w", start));

  const auto active_command = core.snapshot(start + std::chrono::milliseconds(200), 0.35);
  EXPECT_DOUBLE_EQ(active_command.linear_x, 1.0);
  ASSERT_EQ(active_command.active_keys.size(), 1U);
  EXPECT_EQ(active_command.active_keys.front(), "w");

  const auto released_command = core.snapshot(start + std::chrono::milliseconds(400), 0.35);
  EXPECT_DOUBLE_EQ(released_command.linear_x, 0.0);
  EXPECT_TRUE(released_command.active_keys.empty());
}
