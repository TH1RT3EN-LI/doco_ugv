#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ugv_teleop/keyboard_teleop_common.hpp"

namespace ugv_teleop
{

class KeyboardTtyTeleopNode : public rclcpp::Node
{
public:
  KeyboardTtyTeleopNode()
  : rclcpp::Node("keyboard_tty_teleop_node")
  {
    common_params_ = declare_common_parameters(*this);
    core_ = create_core_from_params(common_params_);
    publisher_ = create_twist_publisher_adapter(*this, common_params_);

    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;
    repeat_timeout_sec_ = this->declare_parameter<double>(
      "repeat_timeout_sec", 0.75, read_only_descriptor);
    read_poll_timeout_ = this->declare_parameter<double>(
      "read_poll_timeout", 0.02, read_only_descriptor);
    tty_device_path_ = this->declare_parameter<std::string>(
      "tty_device_path", "", read_only_descriptor);
    read_poll_timeout_ = std::max(0.001, read_poll_timeout_);

    const auto publish_period = std::chrono::duration<double>(
      1.0 / std::max(1.0, common_params_.publish_rate));
    publish_timer_ = this->create_wall_timer(
      publish_period,
      std::bind(&KeyboardTtyTeleopNode::publish_timer_callback, this));

    input_thread_ = std::thread(&KeyboardTtyTeleopNode::input_thread_main, this);
  }

  ~KeyboardTtyTeleopNode() override
  {
    shutdown();
  }

  void shutdown()
  {
    if (shutdown_done_.exchange(true)) {
      return;
    }
    stop_requested_.store(true);
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
    core_->emergency_stop(std::chrono::steady_clock::now());
    if (rclcpp::ok()) {
      publisher_->publish_zero(core_.get());
    }
  }

private:
  static std::string resolve_tty_path(const std::string & explicit_path)
  {
    std::vector<std::string> candidates;
    if (!explicit_path.empty()) {
      candidates.push_back(explicit_path);
    }

    char * tty_name = ttyname(STDIN_FILENO);
    if (tty_name != nullptr) {
      candidates.emplace_back(tty_name);
    }
    candidates.emplace_back("/dev/tty");

    for (const auto & candidate : candidates) {
      const int fd = open(candidate.c_str(), O_RDONLY | O_NONBLOCK);
      if (fd < 0) {
        continue;
      }
      const bool is_tty = isatty(fd) == 1;
      close(fd);
      if (is_tty) {
        return candidate;
      }
    }

    throw std::runtime_error("No interactive tty available");
  }

  void publish_timer_callback()
  {
    const auto command = core_->snapshot(
      std::chrono::steady_clock::now(),
      repeat_timeout_sec_);
    publisher_->publish(command);
  }

  void handle_input_char(char ch, const std::chrono::steady_clock::time_point & now)
  {
    if (ch == '\x03') {
      stop_requested_.store(true);
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
      return;
    }
    const char lowered = static_cast<char>(
      std::tolower(static_cast<unsigned char>(ch)));
    core_->handle_key_press(std::string(1, lowered), now);
  }

  void input_thread_main()
  {
    int tty_fd = -1;
    termios old_termios{};
    bool termios_saved = false;

    try {
      const std::string tty_path = resolve_tty_path(tty_device_path_);
      tty_fd = open(tty_path.c_str(), O_RDONLY | O_NONBLOCK);
      if (tty_fd < 0) {
        throw std::runtime_error("Failed to open tty device");
      }
      if (tcgetattr(tty_fd, &old_termios) != 0) {
        throw std::runtime_error("Failed to read tty attributes");
      }
      termios_saved = true;

      termios raw_mode = old_termios;
      cfmakeraw(&raw_mode);
      if (tcsetattr(tty_fd, TCSADRAIN, &raw_mode) != 0) {
        throw std::runtime_error("Failed to switch tty to raw mode");
      }
      RCLCPP_INFO(this->get_logger(), "keyboard tty teleop attached to %s", tty_path.c_str());

      while (rclcpp::ok() && !stop_requested_.load()) {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(tty_fd, &read_set);
        timeval timeout{};
        timeout.tv_sec = static_cast<long>(read_poll_timeout_);
        timeout.tv_usec = static_cast<long>((read_poll_timeout_ - timeout.tv_sec) * 1e6);

        const int ready = select(tty_fd + 1, &read_set, nullptr, nullptr, &timeout);
        if (ready <= 0) {
          continue;
        }
        if (!FD_ISSET(tty_fd, &read_set)) {
          continue;
        }

        char buffer[64];
        const ssize_t bytes_read = read(tty_fd, buffer, sizeof(buffer));
        if (bytes_read <= 0) {
          continue;
        }
        const auto now = std::chrono::steady_clock::now();
        for (ssize_t i = 0; i < bytes_read; ++i) {
          handle_input_char(buffer[i], now);
        }
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        this->get_logger(),
        "keyboard tty teleop cannot open interactive terminal: %s",
        exception.what());
      stop_requested_.store(true);
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }

    if (tty_fd >= 0 && termios_saved) {
      (void)tcsetattr(tty_fd, TCSADRAIN, &old_termios);
    }
    if (tty_fd >= 0) {
      close(tty_fd);
    }
  }

  CommonTeleopParams common_params_;
  std::shared_ptr<KeyboardTeleopCore> core_;
  std::shared_ptr<TwistPublisherAdapter> publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  double repeat_timeout_sec_{0.75};
  double read_poll_timeout_{0.02};
  std::string tty_device_path_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> shutdown_done_{false};
  std::thread input_thread_;
};

}

int main(int argc, char ** argv)
{
  std::cout << ugv_teleop::k_common_keyboard_message << std::endl;
  std::cout << "TTY mode infers key release from key-repeat timeout. Exit with Ctrl-C." <<
    std::endl;

  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    auto node = std::make_shared<ugv_teleop::KeyboardTtyTeleopNode>();
    rclcpp::spin(node);
    node->shutdown();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("keyboard_tty_teleop_node"),
      "Unhandled exception: %s",
      exception.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
