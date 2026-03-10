#pragma once

#include "sensor_msgs/msg/joy.hpp"

namespace ugv_teleop
{

struct JoyAxisSelectorConfig
{
  int dpad_x_axis{7};
  int dpad_y_axis{6};
  int stick_x_axis{1};
  int stick_y_axis{0};
  double deadzone{0.1};
};

sensor_msgs::msg::Joy apply_axis_selection(
  const sensor_msgs::msg::Joy & input,
  const JoyAxisSelectorConfig & config);

}
