#include "ugv_teleop/keyboard_teleop_common.hpp"

#include <algorithm>
#include <stdexcept>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace ugv_teleop
{

const char * const k_common_keyboard_message =
  R"(Key layout:
   Q    W    E
   A    S    D
        X

W/S : forward/backward (linear x)
A/D : left/right (linear y)
Q/E : yaw left/right (angular z)
X   : emergency stop

I/K : increase/decrease linear speed
O/L : increase/decrease angular speed
)";

TwistPublisherAdapter::TwistPublisherAdapter(
  rclcpp::Node & node,
  const std::string & topic,
  bool stamped,
  const std::string & frame_id)
: node_(node), stamped_(stamped), frame_id_(frame_id)
{
  if (stamped_) {
    twist_stamped_publisher_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(topic, 10);
    twist_stamped_msg_.header.frame_id = frame_id_;
  } else {
    twist_publisher_ = node_.create_publisher<geometry_msgs::msg::Twist>(topic, 10);
  }
}

void TwistPublisherAdapter::publish(const TeleopCommand & command)
{
  if (stamped_) {
    auto & twist = twist_stamped_msg_.twist;
    twist_stamped_msg_.header.stamp = node_.get_clock()->now();
    twist.linear.x = command.linear_x;
    twist.linear.y = command.linear_y;
    twist.linear.z = command.linear_z;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = command.angular_z;
    twist_stamped_publisher_->publish(twist_stamped_msg_);
  } else {
    twist_msg_.linear.x = command.linear_x;
    twist_msg_.linear.y = command.linear_y;
    twist_msg_.linear.z = command.linear_z;
    twist_msg_.angular.x = 0.0;
    twist_msg_.angular.y = 0.0;
    twist_msg_.angular.z = command.angular_z;
    twist_publisher_->publish(twist_msg_);
  }
}

void TwistPublisherAdapter::publish_zero(const KeyboardTeleopCore * core)
{
  const auto status = core ? core->status() : KeyboardTeleopStatus{};
  TeleopCommand command;
  command.linear_x = 0.0;
  command.linear_y = 0.0;
  command.linear_z = 0.0;
  command.angular_z = 0.0;
  command.linear_speed = status.linear_speed;
  command.angular_speed = status.angular_speed;
  command.active_keys.clear();
  publish(command);
}

CommonTeleopParams declare_common_parameters(rclcpp::Node & node)
{
  rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
  read_only_descriptor.read_only = true;

  CommonTeleopParams params;
  params.stamped = node.declare_parameter<bool>("stamped", false, read_only_descriptor);
  params.frame_id = node.declare_parameter<std::string>("frame_id", "", read_only_descriptor);
  params.linear_speed = node.declare_parameter<double>("linear_speed", 0.5, read_only_descriptor);
  params.angular_speed = node.declare_parameter<double>("angular_speed", 1.0, read_only_descriptor);
  params.speed_step = node.declare_parameter<double>("speed_step", 0.05, read_only_descriptor);
  params.turn_step = node.declare_parameter<double>("turn_step", 0.1, read_only_descriptor);
  params.cmd_vel_topic = node.declare_parameter<std::string>(
    "cmd_vel_topic", "/ugv/cmd_vel_keyboard", read_only_descriptor);
  params.publish_rate = node.declare_parameter<double>("publish_rate", 60.0, read_only_descriptor);
  params.accel_limit_linear = node.declare_parameter<double>(
    "accel_limit_linear", 2.0, read_only_descriptor);
  params.decel_limit_linear = node.declare_parameter<double>(
    "decel_limit_linear", 3.0, read_only_descriptor);
  params.accel_limit_angular = node.declare_parameter<double>(
    "accel_limit_angular", 6.0, read_only_descriptor);
  params.decel_limit_angular = node.declare_parameter<double>(
    "decel_limit_angular", 8.0, read_only_descriptor);
  params.idle_timeout_sec = node.declare_parameter<double>(
    "idle_timeout_sec", 0.0, read_only_descriptor);

  if (!params.stamped && !params.frame_id.empty()) {
    throw std::runtime_error("'frame_id' can only be set when 'stamped' is true");
  }
  return params;
}

std::shared_ptr<KeyboardTeleopCore> create_core_from_params(const CommonTeleopParams & params)
{
  return std::make_shared<KeyboardTeleopCore>(
    params.linear_speed,
    params.angular_speed,
    params.speed_step,
    params.turn_step,
    params.accel_limit_linear,
    params.decel_limit_linear,
    params.accel_limit_angular,
    params.decel_limit_angular,
    params.idle_timeout_sec);
}

std::shared_ptr<TwistPublisherAdapter> create_twist_publisher_adapter(
  rclcpp::Node & node, const CommonTeleopParams & params)
{
  return std::make_shared<TwistPublisherAdapter>(
    node,
    params.cmd_vel_topic,
    params.stamped,
    params.frame_id);
}

}
