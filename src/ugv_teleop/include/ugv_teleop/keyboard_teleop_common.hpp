#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ugv_teleop/keyboard_teleop_core.hpp"

namespace ugv_teleop
{

extern const char * const k_common_keyboard_message;

struct CommonTeleopParams
{
  bool stamped{false};
  std::string frame_id;
  double linear_speed{0.5};
  double angular_speed{1.0};
  double speed_step{0.05};
  double turn_step{0.1};
  std::string cmd_vel_topic{"/ugv/cmd_vel_keyboard"};
  double publish_rate{60.0};
  double accel_limit_linear{2.0};
  double decel_limit_linear{3.0};
  double accel_limit_angular{6.0};
  double decel_limit_angular{8.0};
  double idle_timeout_sec{0.0};
};

class TwistPublisherAdapter
{
public:
  TwistPublisherAdapter(
    rclcpp::Node & node,
    const std::string & topic,
    bool stamped,
    const std::string & frame_id);

  void publish(const TeleopCommand & command);
  void publish_zero(const KeyboardTeleopCore * core = nullptr);

private:
  rclcpp::Node & node_;
  bool stamped_{false};
  std::string frame_id_;
  geometry_msgs::msg::Twist twist_msg_;
  geometry_msgs::msg::TwistStamped twist_stamped_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
};

CommonTeleopParams declare_common_parameters(rclcpp::Node & node);
std::shared_ptr<KeyboardTeleopCore> create_core_from_params(const CommonTeleopParams & params);
std::shared_ptr<TwistPublisherAdapter> create_twist_publisher_adapter(
  rclcpp::Node & node, const CommonTeleopParams & params);

}
