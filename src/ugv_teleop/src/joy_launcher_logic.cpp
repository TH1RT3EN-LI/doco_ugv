#include "ugv_teleop/joy_launcher_logic.hpp"

#include <algorithm>

namespace ugv_teleop
{

std::unordered_map<int, std::string> parse_button_commands(
  const std::vector<std::string> & raw_bindings,
  std::vector<std::string> * warnings)
{
  std::unordered_map<int, std::string> parsed;
  for (const auto & entry : raw_bindings) {
    const auto separator = entry.find(':');
    if (separator == std::string::npos || separator == 0) {
      if (warnings != nullptr) {
        warnings->push_back(
          "Invalid bindings entry \"" + entry + "\", expected \"button:command\".");
      }
      continue;
    }

    try {
      const int button = std::stoi(entry.substr(0, separator));
      const std::string command = entry.substr(separator + 1);
      parsed[button] = command;
    } catch (const std::exception &) {
      if (warnings != nullptr) {
        warnings->push_back(
          "Invalid bindings entry \"" + entry + "\", expected \"button:command\".");
      }
    }
  }
  return parsed;
}

std::unordered_map<int, double> parse_button_floats(
  const std::vector<std::string> & raw_bindings,
  std::vector<std::string> * warnings)
{
  std::unordered_map<int, double> parsed;
  for (const auto & entry : raw_bindings) {
    const auto separator = entry.find(':');
    if (separator == std::string::npos || separator == 0 || separator == (entry.size() - 1)) {
      if (warnings != nullptr) {
        warnings->push_back(
          "Invalid hold_bindings entry \"" + entry + "\", expected \"button:value\".");
      }
      continue;
    }

    try {
      const int button = std::stoi(entry.substr(0, separator));
      const double value = std::max(0.0, std::stod(entry.substr(separator + 1)));
      parsed[button] = value;
    } catch (const std::exception &) {
      if (warnings != nullptr) {
        warnings->push_back(
          "Invalid hold_bindings entry \"" + entry + "\", expected \"button:value\".");
      }
    }
  }
  return parsed;
}

double hold_seconds_for_button(
  int button_idx,
  const std::unordered_map<int, double> & button_hold_secs,
  double default_hold_sec)
{
  const auto it = button_hold_secs.find(button_idx);
  if (it != button_hold_secs.end()) {
    return it->second;
  }
  return default_hold_sec;
}

double clamp_intensity(double intensity)
{
  return std::clamp(intensity, 0.0, 1.0);
}

HoldTriggerResult ButtonHoldTracker::update(
  int button_idx,
  bool current_pressed,
  bool prev_pressed,
  double now_sec,
  double hold_sec)
{
  if (current_pressed && !prev_pressed) {
    button_press_start_ts_[button_idx] = now_sec;
    button_fired_while_held_.erase(button_idx);
  }

  if (current_pressed && button_fired_while_held_.count(button_idx) == 0U) {
    const auto start_it = button_press_start_ts_.find(button_idx);
    const double start_sec =
      (start_it != button_press_start_ts_.end()) ? start_it->second : now_sec;
    const double held_sec = now_sec - start_sec;
    if (held_sec >= hold_sec) {
      button_fired_while_held_.insert(button_idx);
      return HoldTriggerResult{true, held_sec};
    }
  }

  if (!current_pressed) {
    clear_button(button_idx);
  }

  return HoldTriggerResult{};
}

void ButtonHoldTracker::clear_button(int button_idx)
{
  button_press_start_ts_.erase(button_idx);
  button_fired_while_held_.erase(button_idx);
}

void ButtonHoldTracker::clear()
{
  button_press_start_ts_.clear();
  button_fired_while_held_.clear();
}

}
