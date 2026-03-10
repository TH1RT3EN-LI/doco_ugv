#include <algorithm>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "ugv_teleop/joy_launcher_logic.hpp"

namespace ugv_teleop
{

using namespace std::chrono_literals;

class JoyLauncherNode : public rclcpp::Node
{
public:
  JoyLauncherNode()
  : rclcpp::Node("joy_launcher_node")
  {
    this->declare_parameter<std::vector<std::string>>("bindings", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("hold_bindings", std::vector<std::string>{});
    this->declare_parameter<double>("default_hold_sec", 0.0);
    this->declare_parameter<std::vector<int64_t>>(
      "trigger_feedback_buttons",
      std::vector<int64_t>{});
    this->declare_parameter<std::vector<int64_t>>(
      "success_feedback_buttons",
      std::vector<int64_t>{});
    this->declare_parameter<std::string>("feedback_topic", "/ugv/joy/set_feedback");
    this->declare_parameter<double>("trigger_feedback_intensity", 0.45);
    this->declare_parameter<double>("trigger_feedback_duration_sec", 0.15);
    this->declare_parameter<double>("success_feedback_intensity", 1.0);
    this->declare_parameter<double>("success_feedback_duration_sec", 0.35);
    this->declare_parameter<double>("feedback_stop_check_period_sec", 0.02);
    this->declare_parameter<double>("process_poll_period_sec", 0.10);

    const auto bindings = this->get_parameter("bindings").as_string_array();
    const auto hold_bindings = this->get_parameter("hold_bindings").as_string_array();
    std::vector<std::string> warnings;
    button_commands_ = parse_button_commands(bindings, &warnings);
    button_hold_secs_ = parse_button_floats(hold_bindings, &warnings);
    for (const auto & warning : warnings) {
      RCLCPP_WARN(this->get_logger(), "%s", warning.c_str());
    }

    default_hold_sec_ = std::max(0.0, this->get_parameter("default_hold_sec").as_double());

    const auto trigger_buttons =
      this->get_parameter("trigger_feedback_buttons").as_integer_array();
    for (const auto raw_button : trigger_buttons) {
      trigger_feedback_buttons_.insert(static_cast<int>(raw_button));
    }
    const auto success_buttons =
      this->get_parameter("success_feedback_buttons").as_integer_array();
    for (const auto raw_button : success_buttons) {
      success_feedback_buttons_.insert(static_cast<int>(raw_button));
    }

    feedback_topic_ = this->get_parameter("feedback_topic").as_string();
    trigger_feedback_intensity_ = clamp_intensity(
      this->get_parameter("trigger_feedback_intensity").as_double());
    success_feedback_intensity_ = clamp_intensity(
      this->get_parameter("success_feedback_intensity").as_double());
    trigger_feedback_duration_sec_ = std::max(
      0.0, this->get_parameter("trigger_feedback_duration_sec").as_double());
    success_feedback_duration_sec_ = std::max(
      0.0, this->get_parameter("success_feedback_duration_sec").as_double());
    feedback_stop_check_period_sec_ = std::max(
      0.01, this->get_parameter("feedback_stop_check_period_sec").as_double());
    process_poll_period_sec_ = std::max(
      0.05, this->get_parameter("process_poll_period_sec").as_double());

    feedback_pub_ = this->create_publisher<sensor_msgs::msg::JoyFeedback>(
      feedback_topic_, 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/ugv/joy",
      10,
      std::bind(&JoyLauncherNode::joy_callback, this, std::placeholders::_1));
    process_poll_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(process_poll_period_sec_),
      std::bind(&JoyLauncherNode::poll_running_processes, this));
    feedback_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(feedback_stop_check_period_sec_),
      std::bind(&JoyLauncherNode::feedback_timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "JoyLauncher ready, default_hold_sec: %.2f",
      default_hold_sec_);
    RCLCPP_INFO(
      this->get_logger(),
      "Feedback topic: %s",
      feedback_topic_.c_str());
  }

  ~JoyLauncherNode() override
  {
    shutdown();
  }

  void shutdown()
  {
    if (shutdown_done_) {
      return;
    }
    shutdown_done_ = true;

    for (const auto & [button, pid] : running_pids_) {
      (void)button;
      stop_process_tree(pid);
      reap_process(pid);
    }
    running_pids_.clear();
    publish_rumble(0.0, 0.0);
  }

private:
  static double now_sec()
  {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
  }

  static std::optional<pid_t> start_shell_process(const std::string & command)
  {
    const pid_t pid = fork();
    if (pid < 0) {
      return std::nullopt;
    }

    if (pid == 0) {
      if (setsid() < 0) {
        _exit(127);
      }

      const int devnull_fd = open("/dev/null", O_WRONLY);
      if (devnull_fd >= 0) {
        (void)dup2(devnull_fd, STDOUT_FILENO);
        (void)dup2(devnull_fd, STDERR_FILENO);
        (void)close(devnull_fd);
      }

      execl("/bin/bash", "bash", "-lc", command.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }

    return pid;
  }

  static bool is_process_alive(pid_t pid)
  {
    if (pid <= 0) {
      return false;
    }
    if (kill(pid, 0) == 0) {
      return true;
    }
    return errno != ESRCH;
  }

  static void stop_process_tree(pid_t pid)
  {
    if (pid <= 0) {
      return;
    }
    if (!is_process_alive(pid)) {
      return;
    }

    (void)kill(-pid, SIGTERM);
    const auto term_deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < term_deadline) {
      int status = 0;
      const pid_t result = waitpid(pid, &status, WNOHANG);
      if (result == pid) {
        return;
      }
      if (result < 0 && errno == ECHILD) {
        return;
      }
      std::this_thread::sleep_for(20ms);
    }

    (void)kill(-pid, SIGKILL);
    const auto kill_deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < kill_deadline) {
      int status = 0;
      const pid_t result = waitpid(pid, &status, WNOHANG);
      if (result == pid) {
        return;
      }
      if (result < 0 && errno == ECHILD) {
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
  }

  static void reap_process(pid_t pid)
  {
    int status = 0;
    while (waitpid(pid, &status, WNOHANG) > 0) {
    }
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (prev_buttons_.empty()) {
      prev_buttons_.assign(msg->buttons.size(), 0);
      return;
    }

    const double now_seconds = now_sec();
    for (const auto & [button, command] : button_commands_) {
      if (button < 0) {
        continue;
      }
      if (
        button >= static_cast<int>(msg->buttons.size()) ||
        button >= static_cast<int>(prev_buttons_.size()))
      {
        continue;
      }

      const bool current_pressed = msg->buttons[static_cast<std::size_t>(button)] == 1;
      const bool prev_pressed = prev_buttons_[static_cast<std::size_t>(button)] == 1;
      const double hold_sec = hold_seconds_for_button(button, button_hold_secs_, default_hold_sec_);
      const auto trigger = hold_tracker_.update(
        button,
        current_pressed,
        prev_pressed,
        now_seconds,
        hold_sec);
      if (trigger.triggered) {
        trigger_button_command(button, command, hold_sec, trigger.held_sec);
      }
    }

    prev_buttons_ = msg->buttons;
  }

  void trigger_button_command(
    int button,
    const std::string & command,
    double hold_sec,
    double held_sec)
  {
    if (hold_sec > 0.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Button %d long-press %.2fs/%.2fs -> %s",
        button,
        held_sec,
        hold_sec,
        command.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Button %d pressed -> %s",
        button,
        command.c_str());
    }

    if (trigger_feedback_buttons_.count(button) > 0U) {
      publish_rumble(trigger_feedback_intensity_, trigger_feedback_duration_sec_);
    }

    const auto running_it = running_pids_.find(button);
    if (running_it != running_pids_.end()) {
      if (is_process_alive(running_it->second)) {
        RCLCPP_INFO(
          this->get_logger(),
          "Killing previous process for button %d",
          button);
        stop_process_tree(running_it->second);
        reap_process(running_it->second);
      }
      running_pids_.erase(running_it);
    }

    const auto pid = start_shell_process(command);
    if (!pid.has_value()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to start command for button %d: %s",
        button,
        std::strerror(errno));
      return;
    }
    running_pids_[button] = pid.value();
  }

  void poll_running_processes()
  {
    for (auto it = running_pids_.begin(); it != running_pids_.end(); ) {
      const int button = it->first;
      const pid_t pid = it->second;
      int status = 0;
      const pid_t result = waitpid(pid, &status, WNOHANG);
      if (result == 0) {
        ++it;
        continue;
      }

      if (result < 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Failed polling command for button %d: %s",
          button,
          std::strerror(errno));
        it = running_pids_.erase(it);
        continue;
      }

      if (WIFEXITED(status)) {
        const int exit_code = WEXITSTATUS(status);
        if (exit_code == 0) {
          RCLCPP_INFO(
            this->get_logger(),
            "Command for button %d finished successfully",
            button);
          if (success_feedback_buttons_.count(button) > 0U) {
            publish_rumble(success_feedback_intensity_, success_feedback_duration_sec_);
          }
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "Command for button %d exited with code %d",
            button,
            exit_code);
        }
      } else if (WIFSIGNALED(status)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Command for button %d killed by signal %d",
          button,
          WTERMSIG(status));
      }

      it = running_pids_.erase(it);
    }
  }

  void publish_rumble(double intensity, double duration_sec)
  {
    if (!rclcpp::ok()) {
      return;
    }

    const float rumble_intensity = static_cast<float>(clamp_intensity(intensity));
    for (uint8_t rumble_id : {0, 1}) {
      sensor_msgs::msg::JoyFeedback feedback;
      feedback.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
      feedback.id = rumble_id;
      feedback.intensity = rumble_intensity;
      feedback_pub_->publish(feedback);
    }

    if (duration_sec > 0.0) {
      const auto duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(duration_sec));
      feedback_stop_deadline_ =
        std::chrono::steady_clock::now() + duration;
    } else {
      feedback_stop_deadline_.reset();
    }
  }

  void feedback_timer_callback()
  {
    if (!feedback_stop_deadline_.has_value()) {
      return;
    }
    if (std::chrono::steady_clock::now() >= feedback_stop_deadline_.value()) {
      feedback_stop_deadline_.reset();
      publish_rumble(0.0, 0.0);
    }
  }

  std::unordered_map<int, std::string> button_commands_;
  std::unordered_map<int, double> button_hold_secs_;
  double default_hold_sec_{0.0};
  std::unordered_set<int> trigger_feedback_buttons_;
  std::unordered_set<int> success_feedback_buttons_;
  std::string feedback_topic_;
  double trigger_feedback_intensity_{0.45};
  double trigger_feedback_duration_sec_{0.15};
  double success_feedback_intensity_{1.0};
  double success_feedback_duration_sec_{0.35};
  double feedback_stop_check_period_sec_{0.02};
  double process_poll_period_sec_{0.10};

  std::vector<int32_t> prev_buttons_;
  std::unordered_map<int, pid_t> running_pids_;
  ButtonHoldTracker hold_tracker_;
  std::optional<std::chrono::steady_clock::time_point> feedback_stop_deadline_;
  bool shutdown_done_{false};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr process_poll_timer_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    auto node = std::make_shared<ugv_teleop::JoyLauncherNode>();
    rclcpp::spin(node);
    node->shutdown();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("joy_launcher_node"),
      "Unhandled exception: %s",
      exception.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
