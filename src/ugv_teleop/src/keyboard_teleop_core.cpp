#include "ugv_teleop/keyboard_teleop_core.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>

namespace ugv_teleop
{

KeyboardTeleopCore::KeyboardTeleopCore(
  double linear_speed,
  double angular_speed,
  double speed_step,
  double turn_step,
  double accel_limit_linear,
  double decel_limit_linear,
  double accel_limit_angular,
  double decel_limit_angular,
  double idle_timeout_sec)
: linear_speed_(std::max(0.0, linear_speed)),
  angular_speed_(std::max(0.0, angular_speed)),
  speed_step_(std::max(0.0, speed_step)),
  turn_step_(std::max(0.0, turn_step)),
  accel_limit_linear_(std::max(1e-6, accel_limit_linear)),
  decel_limit_linear_(std::max(1e-6, decel_limit_linear)),
  accel_limit_angular_(std::max(1e-6, accel_limit_angular)),
  decel_limit_angular_(std::max(1e-6, decel_limit_angular)),
  idle_timeout_sec_(std::max(0.0, idle_timeout_sec)),
  last_input_time_(std::chrono::steady_clock::now())
{
}

bool KeyboardTeleopCore::handle_key_press(const std::string & key, std::optional<TimePoint> now)
{
  const auto normalized = normalize_key(key);
  if (!normalized.has_value()) {
    return false;
  }

  const auto timestamp = coerce_now(now);
  std::lock_guard<std::mutex> lock(mutex_);
  last_input_time_ = timestamp;

  if (normalized.value() == "x") {
    clear_move_keys_locked();
    zero_command_locked();
    last_update_time_ = timestamp;
    return true;
  }

  const auto & move_binding_map = move_bindings();
  if (move_binding_map.find(normalized.value()) != move_binding_map.end()) {
    pressed_move_keys_.insert(normalized.value());
    move_key_last_seen_[normalized.value()] = timestamp;
    return true;
  }

  const auto & speed_binding_map = speed_bindings();
  const auto speed_it = speed_binding_map.find(normalized.value());
  if (speed_it != speed_binding_map.end()) {
    linear_speed_ = std::max(
      0.0, linear_speed_ + speed_it->second.linear_step_direction * speed_step_);
    angular_speed_ = std::max(
      0.0, angular_speed_ + speed_it->second.angular_step_direction * turn_step_);
    return true;
  }

  return false;
}

bool KeyboardTeleopCore::handle_key_release(const std::string & key, std::optional<TimePoint> now)
{
  const auto normalized = normalize_key(key);
  if (!normalized.has_value()) {
    return false;
  }

  const auto & move_binding_map = move_bindings();
  if (move_binding_map.find(normalized.value()) == move_binding_map.end()) {
    return false;
  }

  const auto timestamp = coerce_now(now);
  std::lock_guard<std::mutex> lock(mutex_);
  last_input_time_ = timestamp;
  if (normalized.value() != "x") {
    pressed_move_keys_.erase(normalized.value());
    move_key_last_seen_.erase(normalized.value());
  }
  return true;
}

void KeyboardTeleopCore::clear_move_keys(std::optional<TimePoint> now)
{
  const auto timestamp = coerce_now(now);
  std::lock_guard<std::mutex> lock(mutex_);
  clear_move_keys_locked();
  last_input_time_ = timestamp;
}

void KeyboardTeleopCore::emergency_stop(std::optional<TimePoint> now)
{
  const auto timestamp = coerce_now(now);
  std::lock_guard<std::mutex> lock(mutex_);
  clear_move_keys_locked();
  zero_command_locked();
  last_input_time_ = timestamp;
  last_update_time_ = timestamp;
}

void KeyboardTeleopCore::expire_stale_move_keys(
  double stale_after_sec,
  std::optional<TimePoint> now)
{
  const auto timeout = std::max(0.0, stale_after_sec);
  const auto timestamp = coerce_now(now);
  std::lock_guard<std::mutex> lock(mutex_);
  expire_stale_locked(timestamp, timeout);
}

TeleopCommand KeyboardTeleopCore::snapshot(
  std::optional<TimePoint> now,
  std::optional<double> stale_key_timeout_sec)
{
  const auto timestamp = coerce_now(now);
  std::lock_guard<std::mutex> lock(mutex_);

  if (stale_key_timeout_sec.has_value()) {
    expire_stale_locked(timestamp, std::max(0.0, stale_key_timeout_sec.value()));
  }

  const auto idle_elapsed =
    std::chrono::duration<double>(timestamp - last_input_time_).count();
  if (idle_timeout_sec_ > 0.0 && idle_elapsed > idle_timeout_sec_) {
    clear_move_keys_locked();
  }

  const auto target = target_vector_locked();
  const double dt = last_update_time_.has_value() ?
    std::max(
    0.0,
    std::chrono::duration<double>(timestamp - last_update_time_.value()).count()) : 0.0;

  const double target_linear_x = target.x * linear_speed_;
  const double target_linear_y = target.y * linear_speed_;
  const double target_linear_z = target.z * linear_speed_;
  const double target_angular_z = target.th * angular_speed_;

  current_linear_x_ = slew_axis(
    current_linear_x_,
    target_linear_x,
    dt,
    accel_limit_linear_,
    decel_limit_linear_);
  current_linear_y_ = slew_axis(
    current_linear_y_,
    target_linear_y,
    dt,
    accel_limit_linear_,
    decel_limit_linear_);
  current_linear_z_ = slew_axis(
    current_linear_z_,
    target_linear_z,
    dt,
    accel_limit_linear_,
    decel_limit_linear_);
  current_angular_z_ = slew_axis(
    current_angular_z_,
    target_angular_z,
    dt,
    accel_limit_angular_,
    decel_limit_angular_);
  last_update_time_ = timestamp;

  return TeleopCommand{
    current_linear_x_,
    current_linear_y_,
    current_linear_z_,
    current_angular_z_,
    linear_speed_,
    angular_speed_,
    sorted_keys(pressed_move_keys_)};
}

KeyboardTeleopStatus KeyboardTeleopCore::status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return KeyboardTeleopStatus{
    linear_speed_,
    angular_speed_,
    sorted_keys(pressed_move_keys_)};
}

TeleopCommand KeyboardTeleopCore::zero_command()
{
  std::lock_guard<std::mutex> lock(mutex_);
  zero_command_locked();
  return TeleopCommand{
    0.0,
    0.0,
    0.0,
    0.0,
    linear_speed_,
    angular_speed_,
    sorted_keys(pressed_move_keys_)};
}

bool KeyboardTeleopCore::is_move_key(const std::string & key)
{
  const auto normalized = normalize_key(key);
  if (!normalized.has_value()) {
    return false;
  }
  return move_bindings().find(normalized.value()) != move_bindings().end();
}

bool KeyboardTeleopCore::is_speed_key(const std::string & key)
{
  const auto normalized = normalize_key(key);
  if (!normalized.has_value()) {
    return false;
  }
  return speed_bindings().find(normalized.value()) != speed_bindings().end();
}

std::optional<std::string> KeyboardTeleopCore::normalize_key(const std::string & key)
{
  if (key.empty()) {
    return std::nullopt;
  }
  std::string normalized = key;
  std::transform(
    normalized.begin(),
    normalized.end(),
    normalized.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (
    move_bindings().find(normalized) != move_bindings().end() ||
    speed_bindings().find(normalized) != speed_bindings().end())
  {
    return normalized;
  }
  return std::nullopt;
}

double KeyboardTeleopCore::slew_axis(
  double current,
  double target,
  double dt,
  double accel_limit,
  double decel_limit)
{
  if (dt <= 0.0) {
    return target;
  }
  if (std::fabs(current - target) <= 1e-9) {
    return target;
  }

  double limit = accel_limit;
  if (current == 0.0) {
    limit = std::fabs(target) > 0.0 ? accel_limit : decel_limit;
  } else if ((current * target) < 0.0 || std::fabs(target) < std::fabs(current)) {
    limit = decel_limit;
  }

  const double max_delta = limit * dt;
  const double delta = target - current;
  if (std::fabs(delta) <= max_delta) {
    return target;
  }
  return current + std::copysign(max_delta, delta);
}

KeyboardTeleopCore::TimePoint KeyboardTeleopCore::coerce_now(std::optional<TimePoint> now)
{
  return now.has_value() ? now.value() : std::chrono::steady_clock::now();
}

const std::map<std::string, KeyboardTeleopCore::MoveVector> & KeyboardTeleopCore::move_bindings()
{
  static const std::map<std::string, MoveVector> kBindings = {
    {"w", MoveVector{1.0, 0.0, 0.0, 0.0}},
    {"s", MoveVector{-1.0, 0.0, 0.0, 0.0}},
    {"a", MoveVector{0.0, 1.0, 0.0, 0.0}},
    {"d", MoveVector{0.0, -1.0, 0.0, 0.0}},
    {"q", MoveVector{0.0, 0.0, 0.0, 1.0}},
    {"e", MoveVector{0.0, 0.0, 0.0, -1.0}},
    {"x", MoveVector{0.0, 0.0, 0.0, 0.0}},
  };
  return kBindings;
}

const std::map<std::string, KeyboardTeleopCore::SpeedVector> & KeyboardTeleopCore::speed_bindings()
{
  static const std::map<std::string, SpeedVector> kBindings = {
    {"i", SpeedVector{1.0, 0.0}},
    {"k", SpeedVector{-1.0, 0.0}},
    {"o", SpeedVector{0.0, 1.0}},
    {"l", SpeedVector{0.0, -1.0}},
  };
  return kBindings;
}

void KeyboardTeleopCore::clear_move_keys_locked()
{
  pressed_move_keys_.clear();
  move_key_last_seen_.clear();
}

void KeyboardTeleopCore::zero_command_locked()
{
  current_linear_x_ = 0.0;
  current_linear_y_ = 0.0;
  current_linear_z_ = 0.0;
  current_angular_z_ = 0.0;
}

void KeyboardTeleopCore::expire_stale_locked(TimePoint now, double timeout_sec)
{
  std::vector<std::string> stale_keys;
  stale_keys.reserve(move_key_last_seen_.size());
  for (const auto & [key, seen_at] : move_key_last_seen_) {
    const double elapsed = std::chrono::duration<double>(now - seen_at).count();
    if (elapsed > timeout_sec) {
      stale_keys.push_back(key);
    }
  }
  for (const auto & key : stale_keys) {
    pressed_move_keys_.erase(key);
    move_key_last_seen_.erase(key);
  }
}

KeyboardTeleopCore::MoveVector KeyboardTeleopCore::target_vector_locked() const
{
  MoveVector total;
  for (const auto & key : pressed_move_keys_) {
    const auto it = move_bindings().find(key);
    if (it == move_bindings().end()) {
      continue;
    }
    total.x += it->second.x;
    total.y += it->second.y;
    total.z += it->second.z;
    total.th += it->second.th;
  }

  total.x = std::clamp(total.x, -1.0, 1.0);
  total.y = std::clamp(total.y, -1.0, 1.0);
  total.z = std::clamp(total.z, -1.0, 1.0);
  total.th = std::clamp(total.th, -1.0, 1.0);
  return total;
}

std::vector<std::string> KeyboardTeleopCore::sorted_keys(const std::set<std::string> & keys)
{
  return std::vector<std::string>(keys.begin(), keys.end());
}

}
