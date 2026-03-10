#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <glob.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <linux/input.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ugv_teleop/keyboard_teleop_common.hpp"

namespace ugv_teleop
{

namespace
{
constexpr unsigned long k_evioc_grab = 0x40044590;

const std::unordered_map<int, std::string> k_key_code_to_char = {
  {16, "q"},
  {17, "w"},
  {18, "e"},
  {23, "i"},
  {24, "o"},
  {30, "a"},
  {31, "s"},
  {32, "d"},
  {37, "k"},
  {38, "l"},
  {45, "x"},
};

std::vector<std::string> discover_evdev_keyboard_devices(const std::string & explicit_device)
{
  if (!explicit_device.empty()) {
    if (access(explicit_device.c_str(), F_OK) == 0) {
      return {explicit_device};
    }
    return {};
  }

  std::vector<std::string> devices;
  glob_t glob_result;
  std::memset(&glob_result, 0, sizeof(glob_result));
  if (glob("/dev/input/by-path/*-event-kbd", 0, nullptr, &glob_result) == 0) {
    for (std::size_t i = 0; i < glob_result.gl_pathc; ++i) {
      devices.emplace_back(glob_result.gl_pathv[i]);
    }
  }
  globfree(&glob_result);
  std::sort(devices.begin(), devices.end());
  if (!devices.empty()) {
    return devices;
  }

  std::ifstream devices_file("/proc/bus/input/devices");
  if (!devices_file.is_open()) {
    return {};
  }

  std::vector<std::string> fallback;
  std::string line;
  while (std::getline(devices_file, line)) {
    const std::string prefix = "H: Handlers=";
    if (line.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string handlers = line.substr(prefix.size());
    if (handlers.find("kbd") == std::string::npos) {
      continue;
    }
    std::istringstream handler_stream(handlers);
    std::string token;
    while (handler_stream >> token) {
      if (token.rfind("event", 0) == 0) {
        fallback.push_back("/dev/input/" + token);
      }
    }
  }
  std::sort(fallback.begin(), fallback.end());
  fallback.erase(std::unique(fallback.begin(), fallback.end()), fallback.end());
  return fallback;
}

}

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode()
  : rclcpp::Node("keyboard_teleop_node")
  {
    common_params_ = declare_common_parameters(*this);
    core_ = create_core_from_params(common_params_);
    publisher_ = create_twist_publisher_adapter(*this, common_params_);

    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;
    evdev_device_ = this->declare_parameter<std::string>(
      "evdev_device", "", read_only_descriptor);
    grab_device_ = this->declare_parameter<bool>(
      "grab_device", false, read_only_descriptor);
    evdev_poll_timeout_ = this->declare_parameter<double>(
      "evdev_poll_timeout", 0.005, read_only_descriptor);
    evdev_poll_timeout_ = std::max(0.001, evdev_poll_timeout_);

    const auto publish_period = std::chrono::duration<double>(
      1.0 / std::max(1.0, common_params_.publish_rate));
    publish_timer_ = this->create_wall_timer(
      publish_period,
      std::bind(&KeyboardTeleopNode::publish_timer_callback, this));

    const auto discovered = discover_evdev_keyboard_devices(evdev_device_);
    for (const auto & path : discovered) {
      if (access(path.c_str(), R_OK) == 0) {
        readable_devices_.push_back(path);
      }
    }
    if (readable_devices_.empty()) {
      throw std::runtime_error("No readable keyboard evdev device found. Please set evdev_device.");
    }

    keyboard_thread_ = std::thread(&KeyboardTeleopNode::keyboard_thread_main, this);
    RCLCPP_WARN(
      this->get_logger(),
      "Using legacy evdev keyboard backend. This mode may conflict with desktop input.");
  }

  ~KeyboardTeleopNode() override
  {
    shutdown();
  }

  void shutdown()
  {
    if (shutdown_done_.exchange(true)) {
      return;
    }
    stop_requested_.store(true);
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
    core_->emergency_stop(std::chrono::steady_clock::now());
    if (rclcpp::ok()) {
      publisher_->publish_zero(core_.get());
    }
  }

private:
  void publish_timer_callback()
  {
    const auto command = core_->snapshot(std::chrono::steady_clock::now());
    publisher_->publish(command);
  }

  void keyboard_thread_main()
  {
    std::vector<int> fds;
    bool ctrl_pressed = false;

    try {
      for (const auto & path : readable_devices_) {
        const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
          continue;
        }
        if (grab_device_) {
          if (ioctl(fd, k_evioc_grab, 1) < 0) {
            close(fd);
            continue;
          }
        }
        fds.push_back(fd);
      }

      if (fds.empty()) {
        stop_requested_.store(true);
        if (rclcpp::ok()) {
          rclcpp::shutdown();
        }
        return;
      }

      while (rclcpp::ok() && !stop_requested_.load()) {
        fd_set read_set;
        FD_ZERO(&read_set);
        int max_fd = -1;
        for (int fd : fds) {
          FD_SET(fd, &read_set);
          max_fd = std::max(max_fd, fd);
        }

        timeval timeout{};
        timeout.tv_sec = static_cast<long>(evdev_poll_timeout_);
        timeout.tv_usec = static_cast<long>((evdev_poll_timeout_ - timeout.tv_sec) * 1e6);
        const int ready = select(max_fd + 1, &read_set, nullptr, nullptr, &timeout);
        if (ready <= 0) {
          continue;
        }

        for (int fd : fds) {
          if (!FD_ISSET(fd, &read_set)) {
            continue;
          }
          std::array<input_event, 128> events{};
          const ssize_t bytes_read = read(fd, events.data(), sizeof(events));
          if (bytes_read <= 0) {
            continue;
          }
          const std::size_t event_count =
            static_cast<std::size_t>(bytes_read) / sizeof(input_event);
          const auto now = std::chrono::steady_clock::now();

          for (std::size_t i = 0; i < event_count; ++i) {
            const auto & event = events[i];
            if (event.type != EV_KEY) {
              continue;
            }

            if (event.code == KEY_LEFTCTRL || event.code == KEY_RIGHTCTRL) {
              ctrl_pressed = (event.value == 1 || event.value == 2);
              continue;
            }

            if (event.code == KEY_C && event.value == 1 && ctrl_pressed) {
              stop_requested_.store(true);
              if (rclcpp::ok()) {
                rclcpp::shutdown();
              }
              break;
            }

            const auto key_it = k_key_code_to_char.find(event.code);
            if (key_it == k_key_code_to_char.end()) {
              continue;
            }
            const std::string & key = key_it->second;
            if (KeyboardTeleopCore::is_move_key(key)) {
              if (event.value == 0) {
                core_->handle_key_release(key, now);
              } else if (event.value == 1 || event.value == 2) {
                core_->handle_key_press(key, now);
              }
            } else if (KeyboardTeleopCore::is_speed_key(key) && event.value == 1) {
              core_->handle_key_press(key, now);
            }
          }
        }
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(this->get_logger(), "Evdev keyboard thread failed: %s", exception.what());
    }

    core_->clear_move_keys(std::chrono::steady_clock::now());

    for (int fd : fds) {
      if (grab_device_) {
        (void)ioctl(fd, k_evioc_grab, 0);
      }
      close(fd);
    }
  }

  CommonTeleopParams common_params_;
  std::shared_ptr<KeyboardTeleopCore> core_;
  std::shared_ptr<TwistPublisherAdapter> publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string evdev_device_;
  bool grab_device_{false};
  double evdev_poll_timeout_{0.005};
  std::vector<std::string> readable_devices_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> shutdown_done_{false};
  std::thread keyboard_thread_;
};

}

int main(int argc, char ** argv)
{
  std::cout << ugv_teleop::k_common_keyboard_message << std::endl;

  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    auto node = std::make_shared<ugv_teleop::KeyboardTeleopNode>();
    rclcpp::spin(node);
    node->shutdown();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("keyboard_teleop_node"),
      "Unhandled exception: %s",
      exception.what());
    exit_code = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return exit_code;
}
