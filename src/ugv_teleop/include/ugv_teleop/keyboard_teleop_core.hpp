#pragma once

#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace ugv_teleop
{

struct TeleopCommand
{
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_z{0.0};
  double linear_speed{0.0};
  double angular_speed{0.0};
  std::vector<std::string> active_keys;
};

struct KeyboardTeleopStatus
{
  double linear_speed{0.0};
  double angular_speed{0.0};
  std::vector<std::string> active_keys;
};

class KeyboardTeleopCore
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  KeyboardTeleopCore(
    double linear_speed,
    double angular_speed,
    double speed_step,
    double turn_step,
    double accel_limit_linear,
    double decel_limit_linear,
    double accel_limit_angular,
    double decel_limit_angular,
    double idle_timeout_sec);

  bool handle_key_press(const std::string & key, std::optional<TimePoint> now = std::nullopt);
  bool handle_key_release(const std::string & key, std::optional<TimePoint> now = std::nullopt);
  void clear_move_keys(std::optional<TimePoint> now = std::nullopt);
  void emergency_stop(std::optional<TimePoint> now = std::nullopt);
  void expire_stale_move_keys(double stale_after_sec, std::optional<TimePoint> now = std::nullopt);

  TeleopCommand snapshot(
    std::optional<TimePoint> now = std::nullopt,
    std::optional<double> stale_key_timeout_sec = std::nullopt);
  KeyboardTeleopStatus status() const;
  TeleopCommand zero_command();

  static bool is_move_key(const std::string & key);
  static bool is_speed_key(const std::string & key);

private:
  struct MoveVector
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double th{0.0};
  };

  struct SpeedVector
  {
    double linear_step_direction{0.0};
    double angular_step_direction{0.0};
  };

  static std::optional<std::string> normalize_key(const std::string & key);
  static double slew_axis(
    double current, double target, double dt, double accel_limit, double decel_limit);
  static TimePoint coerce_now(std::optional<TimePoint> now);
  static const std::map<std::string, MoveVector> & move_bindings();
  static const std::map<std::string, SpeedVector> & speed_bindings();

  void clear_move_keys_locked();
  void zero_command_locked();
  void expire_stale_locked(TimePoint now, double timeout_sec);
  MoveVector target_vector_locked() const;
  static std::vector<std::string> sorted_keys(const std::set<std::string> & keys);

  mutable std::mutex mutex_;
  std::set<std::string> pressed_move_keys_;
  std::map<std::string, TimePoint> move_key_last_seen_;
  double linear_speed_{0.0};
  double angular_speed_{0.0};
  double speed_step_{0.0};
  double turn_step_{0.0};
  double accel_limit_linear_{1.0};
  double decel_limit_linear_{1.0};
  double accel_limit_angular_{1.0};
  double decel_limit_angular_{1.0};
  double idle_timeout_sec_{0.0};

  double current_linear_x_{0.0};
  double current_linear_y_{0.0};
  double current_linear_z_{0.0};
  double current_angular_z_{0.0};

  TimePoint last_input_time_;
  std::optional<TimePoint> last_update_time_;
};

}
