#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class SafetyWatchdogNode : public rclcpp::Node {
 public:
  SafetyWatchdogNode()
      : rclcpp::Node("safety_watchdog"),
        check_rate_hz_(declare_parameter<double>("check_rate_hz", 50.0)),
        odom_timeout_sec_(declare_parameter<double>("odom_timeout_sec", 2.0)),
        imu_timeout_sec_(declare_parameter<double>("imu_timeout_sec", 2.0)),
        battery_timeout_sec_(declare_parameter<double>("battery_timeout_sec", 5.0)),
        grace_period_sec_(declare_parameter<double>("grace_period_sec", 15.0)),
        enable_odom_check_(declare_parameter<bool>("enable_odom_check", true)),
        enable_imu_check_(declare_parameter<bool>("enable_imu_check", true)),
        enable_battery_check_(declare_parameter<bool>("enable_battery_check", true)),
        enable_battery_voltage_check_(
            declare_parameter<bool>("enable_battery_voltage_check", true)),
        battery_min_voltage_(declare_parameter<double>("battery_min_voltage", 10.0)),
        battery_window_size_(declare_parameter<int>("battery_window_size", 50)),
        battery_hysteresis_voltage_(declare_parameter<double>("battery_hysteresis_voltage", 0.5)),
        enable_tilt_check_(declare_parameter<bool>("enable_tilt_check", true)),
        max_tilt_rad_(DegreesToRadians(declare_parameter<double>("max_tilt_degrees", 45.0))),
        tilt_confirm_count_(declare_parameter<int>("tilt_confirm_count", 20)),
        diag_period_sec_(declare_parameter<double>("diag_period_sec", 1.0)),
        start_time_(now()) {
    auto lock_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    lock_pub_ = create_publisher<std_msgs::msg::Bool>("/ugv/watchdog/lock", lock_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("/ugv/watchdog/status", 10);

    PublishLock(false, true);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/ugv/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr) { last_odom_time_ = now(); });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/ugv/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          last_imu_time_ = now();
          if (enable_tilt_check_ && !triggered_) {
            UpdateTilt(*msg);
          }
        });

    battery_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/ugv/battery_voltage", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) { BatteryCallback(*msg); });

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "/ugv/watchdog/reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          ResetCallback(request, response);
        });

    diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diag_updater_->setHardwareID("duojin01");
    diag_updater_->setPeriod(diag_period_sec_);
    diag_updater_->add("总状态", this, &SafetyWatchdogNode::DiagOverall);
    diag_updater_->add("底盘通信 (odom)", this, &SafetyWatchdogNode::DiagOdom);
    diag_updater_->add("底盘通信 (imu)", this, &SafetyWatchdogNode::DiagImu);
    diag_updater_->add("底盘通信 (battery)", this, &SafetyWatchdogNode::DiagBatteryTopic);
    diag_updater_->add("电池电压", this, &SafetyWatchdogNode::DiagBatteryVoltage);
    diag_updater_->add("IMU 姿态", this, &SafetyWatchdogNode::DiagTilt);

    check_timer_ = rclcpp::create_timer(this, get_clock(),
                                        rclcpp::Duration::from_seconds(1.0 / check_rate_hz_),
                                        [this]() { CheckLoop(); });

    const auto max_tilt_deg = RadiansToDegrees(max_tilt_rad_);
    const auto separator = std::string(60, '=');
    RCLCPP_INFO(get_logger(), "%s", separator.c_str());
    RCLCPP_INFO(get_logger(), "  watchdog 已启动");
    RCLCPP_INFO(get_logger(), "  宽限期: %.1fs", grace_period_sec_);
    RCLCPP_INFO(get_logger(), "  odom 超时: %.1fs (%s)", odom_timeout_sec_,
                enable_odom_check_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "  IMU 超时: %.1fs (%s)", imu_timeout_sec_,
                enable_imu_check_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "  电池超时: %.1fs (%s)", battery_timeout_sec_,
                enable_battery_check_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "  电压保护: <%.1fV 窗口=%d 迟滞=%.1fV (%s)", battery_min_voltage_,
                battery_window_size_, battery_hysteresis_voltage_,
                enable_battery_voltage_check_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "  倾斜保护: >%.0f° 连续%d帧 (%s)", max_tilt_deg, tilt_confirm_count_,
                enable_tilt_check_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(), "%s", separator.c_str());
  }

 private:
  static constexpr const char* kReasonOdomTimeout = "底盘里程计 (/ugv/odom) 超时，疑似串口断联";
  static constexpr const char* kReasonImuTimeout = "IMU (/ugv/imu) 超时，疑似串口断联";
  static constexpr const char* kReasonBatteryTimeout =
      "电池电压 (/ugv/battery_voltage) 超时，疑似串口断联";
  static constexpr const char* kReasonBatteryLow = "电池电压过低 (滑动窗口均值)";
  static constexpr const char* kReasonImuTilt = "IMU 连续检测到姿态异常（倾斜角过大）";

  static double DegreesToRadians(double degrees) { return degrees * M_PI / 180.0; }

  static double RadiansToDegrees(double radians) { return radians * 180.0 / M_PI; }

  static std::string FormatFixed(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
  }

  void PublishLock(bool locked, bool force = false) {
    const auto current_time = now();
    const bool state_changed = !last_published_lock_.has_value() || last_published_lock_.value() != locked;
    const bool republish_due =
        locked && last_lock_publish_time_.has_value() &&
        (current_time - last_lock_publish_time_.value()).seconds() >= 1.0;
    if (!force && !state_changed && !republish_due) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = locked;
    lock_pub_->publish(msg);
    last_published_lock_ = locked;
    last_lock_publish_time_ = current_time;
  }

  void PublishStatus(const std::string& status) {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }

  double SecondsSince(const rclcpp::Time& timestamp) const { return (now() - timestamp).seconds(); }

  bool CheckTopicTimeout(
      const std::optional<rclcpp::Time>& last_message_time,
      double timeout_sec,
      const char* reason,
      const std::string& missing_suffix) {
    if (!last_message_time.has_value()) {
      TriggerEstop(std::string(reason) + missing_suffix);
      return true;
    }

    const double age = SecondsSince(last_message_time.value());
    if (age > timeout_sec) {
      TriggerEstop(std::string(reason) + " (已 " + FormatFixed(age, 1) + "s 未收到)");
      return true;
    }

    return false;
  }

  void UpdateTimeoutDiag(
      diagnostic_updater::DiagnosticStatusWrapper& stat,
      bool enabled,
      const std::optional<rclcpp::Time>& last_message_time,
      double timeout_sec) {
    if (!enabled) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "检查已禁用");
      return;
    }

    if (!last_message_time.has_value()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "等待首条消息");
      return;
    }

    const double age = SecondsSince(last_message_time.value());
    stat.add("最后消息距今 (s)", FormatFixed(age, 2));
    stat.add("超时阈值 (s)", FormatFixed(timeout_sec, 1));
    if (age > timeout_sec) {
      stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "超时 (" + FormatFixed(age, 1) + "s > " + FormatFixed(timeout_sec, 1) + "s)");
    } else if (age > timeout_sec * 0.5) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "接近超时 (" + FormatFixed(age, 1) + "s)");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                   "正常 (" + FormatFixed(age, 2) + "s)");
    }
  }

  void BatteryCallback(const std_msgs::msg::Float32& battery_msg) {
    last_battery_time_ = now();
    const double voltage = battery_msg.data;

    if (voltage > 0.5) {
      battery_window_.push_back(voltage);
      while (battery_window_.size() > static_cast<std::size_t>(battery_window_size_)) {
        battery_window_.pop_front();
      }
    }

    if (battery_window_.size() >= 5U) {
      double sum = 0.0;
      for (const double sample : battery_window_) {
        sum += sample;
      }
      battery_avg_ = sum / static_cast<double>(battery_window_.size());
    } else {
      battery_avg_.reset();
    }

    if (enable_battery_voltage_check_ && !triggered_ && battery_avg_.has_value() &&
        battery_avg_.value() < battery_min_voltage_) {
      std::ostringstream stream;
      stream << kReasonBatteryLow << ": 均值=" << FormatFixed(battery_avg_.value(), 2) << "V < "
             << FormatFixed(battery_min_voltage_, 2) << "V "
             << "(窗口" << battery_window_.size() << "帧)";
      TriggerEstop(stream.str());
    }
  }

  void ResetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (triggered_) {
      RCLCPP_WARN(get_logger(), "watchdog 已被手动重置，解除 twist_mux lock");
      triggered_ = false;
      trigger_reason_.clear();
      trigger_time_.reset();
      last_reminder_time_.reset();
      PublishLock(false, true);
      start_time_ = now();
      last_odom_time_.reset();
      last_imu_time_.reset();
      last_battery_time_.reset();
      last_lock_publish_time_.reset();
      battery_window_.clear();
      battery_avg_.reset();
      tilt_consecutive_count_ = 0;
      response->success = true;
      response->message = "watchdog 已重置，twist_mux lock 已解除";
    } else {
      response->success = true;
      response->message = "watchdog 当前未处于触发状态";
    }
  }

  void CheckLoop() {
    if (triggered_) {
      PublishLock(true);

      const double elapsed = SecondsSince(trigger_time_.value());
      PublishStatus("LOCKED (" + FormatFixed(elapsed, 0) + "s) | " + trigger_reason_);

      const double since_reminder = SecondsSince(last_reminder_time_.value());
      if (since_reminder >= 5.0) {
        last_reminder_time_ = now();
        RCLCPP_WARN(get_logger(),
                    "[紧急停车持续中 %.0fs] %s | 恢复: ros2 service call /ugv/watchdog/reset "
                    "std_srvs/srv/Trigger",
                    elapsed, trigger_reason_.c_str());
      }
      return;
    }

    const double elapsed = SecondsSince(start_time_);
    if (elapsed < grace_period_sec_) {
      PublishStatus("GRACE_PERIOD (" + FormatFixed(elapsed, 0) + "/" +
                    FormatFixed(grace_period_sec_, 0) + "s)");
      return;
    }

    if (enable_odom_check_) {
      if (CheckTopicTimeout(last_odom_time_, odom_timeout_sec_, kReasonOdomTimeout, " (宽限期后仍未收到)")) {
        return;
      }
    }

    if (enable_imu_check_) {
      if (CheckTopicTimeout(last_imu_time_, imu_timeout_sec_, kReasonImuTimeout, " (宽限期后仍未收到)")) {
        return;
      }
    }

    if (enable_battery_check_) {
      if (CheckTopicTimeout(last_battery_time_, battery_timeout_sec_, kReasonBatteryTimeout, " (宽限期后仍未收到)")) {
        return;
      }
    }

    PublishStatus("OK");
  }

  void UpdateTilt(const sensor_msgs::msg::Imu& imu_msg) {
    const auto& orientation = imu_msg.orientation;
    if (orientation.w == 0.0 && orientation.x == 0.0 && orientation.y == 0.0 &&
        orientation.z == 0.0) {
      return;
    }

    const double sinr_cosp = 2.0 * (orientation.w * orientation.x + orientation.y * orientation.z);
    const double cosr_cosp =
        1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (orientation.w * orientation.y - orientation.z * orientation.x);
    sinp = std::clamp(sinp, -1.0, 1.0);
    const double pitch = std::asin(sinp);

    last_roll_deg_ = RadiansToDegrees(roll);
    last_pitch_deg_ = RadiansToDegrees(pitch);

    if (std::abs(roll) > max_tilt_rad_ || std::abs(pitch) > max_tilt_rad_) {
      ++tilt_consecutive_count_;
      if (tilt_consecutive_count_ >= tilt_confirm_count_) {
        std::ostringstream stream;
        stream << kReasonImuTilt << " (roll=" << FormatFixed(last_roll_deg_, 1)
               << "°, pitch=" << FormatFixed(last_pitch_deg_, 1) << "°, 连续"
               << tilt_consecutive_count_ << "帧)";
        TriggerEstop(stream.str());
      }
    } else {
      tilt_consecutive_count_ = 0;
    }
  }

  void TriggerEstop(const std::string& reason) {
    triggered_ = true;
    trigger_reason_ = reason;
    trigger_time_ = now();
    last_reminder_time_ = trigger_time_;

    PublishLock(true, true);

    const auto separator = std::string(60, '!');
    RCLCPP_ERROR(get_logger(), "%s", separator.c_str());
    RCLCPP_ERROR(get_logger(), "!  ██  紧急停车 (twist_mux LOCKED)  ██");
    RCLCPP_ERROR(get_logger(), "!  原因: %s", reason.c_str());
    RCLCPP_ERROR(get_logger(), "!  twist_mux 已锁定，一切上位机速度指令被阻断");
    RCLCPP_ERROR(get_logger(),
                 "!  手动恢复: ros2 service call /ugv/watchdog/reset std_srvs/srv/Trigger");
    RCLCPP_ERROR(get_logger(), "!  下位机可能仍在错误工作，如果车辆仍在运动建议断电急停");
    RCLCPP_ERROR(get_logger(), "%s", separator.c_str());
  }

  void DiagOverall(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (triggered_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "紧急停车: " + trigger_reason_);
    } else {
      const double elapsed = SecondsSince(start_time_);
      if (elapsed < grace_period_sec_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                     "启动宽限期中 (" + FormatFixed(elapsed, 0) + "/" +
                         FormatFixed(grace_period_sec_, 0) + "s)");
      } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "所有检查正常");
      }
    }

    stat.add("触发状态", triggered_ ? "是" : "否");
    stat.add("触发原因", trigger_reason_.empty() ? "无" : trigger_reason_);
  }

  void DiagOdom(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    UpdateTimeoutDiag(stat, enable_odom_check_, last_odom_time_, odom_timeout_sec_);
  }

  void DiagImu(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    UpdateTimeoutDiag(stat, enable_imu_check_, last_imu_time_, imu_timeout_sec_);
  }

  void DiagBatteryTopic(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    UpdateTimeoutDiag(stat, enable_battery_check_, last_battery_time_, battery_timeout_sec_);
  }

  void DiagBatteryVoltage(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!enable_battery_voltage_check_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "检查已禁用");
      return;
    }

    if (!battery_avg_.has_value()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "采样不足, 等待数据");
      stat.add("窗口采样数",
               std::to_string(battery_window_.size()) + "/" + std::to_string(battery_window_size_));
      return;
    }

    stat.add("窗口均值 (V)", FormatFixed(battery_avg_.value(), 2));
    stat.add("最低阈值 (V)", FormatFixed(battery_min_voltage_, 2));
    stat.add("迟滞 (V)", FormatFixed(battery_hysteresis_voltage_, 2));
    stat.add("窗口采样数",
             std::to_string(battery_window_.size()) + "/" + std::to_string(battery_window_size_));
    if (battery_avg_.value() < battery_min_voltage_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                   "欠压! 均值=" + FormatFixed(battery_avg_.value(), 2) + "V < " +
                       FormatFixed(battery_min_voltage_, 2) + "V");
    } else if (battery_avg_.value() < battery_min_voltage_ + battery_hysteresis_voltage_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "电压偏低 均值=" + FormatFixed(battery_avg_.value(), 2) + "V");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                   "正常 均值=" + FormatFixed(battery_avg_.value(), 2) + "V");
    }
  }

  void DiagTilt(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!enable_tilt_check_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "检查已禁用");
      return;
    }

    stat.add("roll (°)", FormatFixed(last_roll_deg_, 1));
    stat.add("pitch (°)", FormatFixed(last_pitch_deg_, 1));
    stat.add("连续超限帧数",
             std::to_string(tilt_consecutive_count_) + "/" + std::to_string(tilt_confirm_count_));
    stat.add("最大允许角度 (°)", FormatFixed(RadiansToDegrees(max_tilt_rad_), 0));

    if (tilt_consecutive_count_ >= tilt_confirm_count_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                   "姿态异常! roll=" + FormatFixed(last_roll_deg_, 1) +
                       "° pitch=" + FormatFixed(last_pitch_deg_, 1) + "°");
    } else if (tilt_consecutive_count_ > 0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "倾斜波动中 (" + std::to_string(tilt_consecutive_count_) + "/" +
                       std::to_string(tilt_confirm_count_) + "帧)");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "姿态正常");
    }
  }

  const double check_rate_hz_;
  const double odom_timeout_sec_;
  const double imu_timeout_sec_;
  const double battery_timeout_sec_;
  const double grace_period_sec_;
  const bool enable_odom_check_;
  const bool enable_imu_check_;
  const bool enable_battery_check_;
  const bool enable_battery_voltage_check_;
  const double battery_min_voltage_;
  const int battery_window_size_;
  const double battery_hysteresis_voltage_;
  const bool enable_tilt_check_;
  const double max_tilt_rad_;
  const int tilt_confirm_count_;
  const double diag_period_sec_;

  bool triggered_ = false;
  std::string trigger_reason_;
  std::optional<rclcpp::Time> trigger_time_;
  std::optional<rclcpp::Time> last_reminder_time_;
  rclcpp::Time start_time_;
  std::optional<bool> last_published_lock_;
  std::optional<rclcpp::Time> last_lock_publish_time_;

  std::optional<rclcpp::Time> last_odom_time_;
  std::optional<rclcpp::Time> last_imu_time_;
  std::optional<rclcpp::Time> last_battery_time_;

  std::deque<double> battery_window_;
  std::optional<double> battery_avg_;

  int tilt_consecutive_count_ = 0;
  double last_roll_deg_ = 0.0;
  double last_pitch_deg_ = 0.0;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lock_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyWatchdogNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
