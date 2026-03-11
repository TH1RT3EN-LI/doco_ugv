#include <gtest/gtest.h>

#include "sensor_msgs/msg/joy.hpp"
#include "ugv_teleop/joy_axis_selector_logic.hpp"

TEST(JoyAxisSelectorLogic, DpadOverridesConfiguredAxes)
{
  sensor_msgs::msg::Joy input;
  input.axes = {0.2F, 0.3F, 0.0F, 0.0F, 0.0F, 0.0F, -1.0F, 1.0F};

  ugv_teleop::JoyAxisSelectorConfig config;
  config.dpad_x_axis = 7;
  config.dpad_y_axis = 6;
  config.stick_x_axis = 1;
  config.stick_y_axis = 0;
  config.deadzone = 0.1;

  const auto output = ugv_teleop::apply_axis_selection(input, config);
  EXPECT_EQ(output.axes[1], 1.0F);
  EXPECT_EQ(output.axes[0], -1.0F);
}

TEST(JoyAxisSelectorLogic, ResizesOutputAxesWhenNeeded)
{
  sensor_msgs::msg::Joy input;
  input.axes = {0.0F};

  ugv_teleop::JoyAxisSelectorConfig config;
  config.dpad_x_axis = 3;
  config.dpad_y_axis = 4;
  config.stick_x_axis = 5;
  config.stick_y_axis = 6;

  const auto output = ugv_teleop::apply_axis_selection(input, config);
  EXPECT_GE(output.axes.size(), 7U);
}
