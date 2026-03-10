#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ugv_teleop
{

std::unordered_map<int, std::string> parse_button_commands(
  const std::vector<std::string> & raw_bindings,
  std::vector<std::string> * warnings = nullptr);

std::unordered_map<int, double> parse_button_floats(
  const std::vector<std::string> & raw_bindings,
  std::vector<std::string> * warnings = nullptr);

double hold_seconds_for_button(
  int button_idx,
  const std::unordered_map<int, double> & button_hold_secs,
  double default_hold_sec);

double clamp_intensity(double intensity);

struct HoldTriggerResult
{
  bool triggered{false};
  double held_sec{0.0};
};

class ButtonHoldTracker
{
public:
  HoldTriggerResult update(
    int button_idx,
    bool current_pressed,
    bool prev_pressed,
    double now_sec,
    double hold_sec);

  void clear_button(int button_idx);
  void clear();

private:
  std::unordered_map<int, double> button_press_start_ts_;
  std::unordered_set<int> button_fired_while_held_;
};

}
