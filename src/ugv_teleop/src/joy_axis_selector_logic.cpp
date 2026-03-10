#include "ugv_teleop/joy_axis_selector_logic.hpp"

#include <algorithm>
#include <cmath>

namespace ugv_teleop
{

sensor_msgs::msg::Joy apply_axis_selection(
  const sensor_msgs::msg::Joy & input,
  const JoyAxisSelectorConfig & config)
{
  sensor_msgs::msg::Joy output;
  output.header = input.header;
  output.buttons = input.buttons;
  output.axes = input.axes;

  const int max_axis = std::max(
    std::max(config.dpad_x_axis, config.dpad_y_axis),
    std::max(config.stick_x_axis, config.stick_y_axis));

  if (max_axis >= 0 && static_cast<int>(output.axes.size()) <= max_axis) {
    output.axes.resize(static_cast<std::size_t>(max_axis + 1), 0.0F);
  }

  const auto axis_value = [&](int axis_index) -> float {
      if (axis_index < 0 || axis_index >= static_cast<int>(input.axes.size())) {
        return 0.0F;
      }
      return input.axes[static_cast<std::size_t>(axis_index)];
    };

  const float dpad_x = axis_value(config.dpad_x_axis);
  const float dpad_y = axis_value(config.dpad_y_axis);
  const bool dpad_active_x = std::fabs(dpad_x) > config.deadzone;
  const bool dpad_active_y = std::fabs(dpad_y) > config.deadzone;

  if (dpad_active_x && config.stick_x_axis >= 0) {
    output.axes[static_cast<std::size_t>(config.stick_x_axis)] = dpad_x;
  }
  if (dpad_active_y && config.stick_y_axis >= 0) {
    output.axes[static_cast<std::size_t>(config.stick_y_axis)] = dpad_y;
  }

  return output;
}

}
